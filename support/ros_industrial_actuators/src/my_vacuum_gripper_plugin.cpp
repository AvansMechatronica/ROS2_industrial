// Copyright 2019 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/*
 * \brief vacuum-gripper-plugin for attracting entities around the model like vacuum
 *
 * \author  Kentaro Wada
 *
 * \date 7 Dec 2015
 */

#include <gazebo/physics/Link.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/physics/PhysicsEngine.hh>
#include <ros_industrial_actuators/my_vacuum_gripper_plugin.hpp>
#include <gazebo/physics/ContactManager.hh>
#include <gazebo/physics/Collision.hh>
#include <gazebo/transport/Subscriber.hh>
#include <gazebo/transport/Node.hh>


#include <gazebo_ros/node.hpp>
#ifdef IGN_PROFILER_ENABLE
#include <ignition/common/Profiler.hh>
#endif
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <sdf/sdf.hh>

#include <algorithm>
#include <memory>
#include <string>
#include <unordered_set>

namespace ros_industrial_actuators
{
class GazeboRosVacuumGripperPrivate
{
public:
  /// Callback to be called at every simulation iteration.
  void OnUpdate();

  /// \brief Function to switch the gripper on/off.
  /// \param[in] req Request
  /// \param[out] res Response
  void OnSwitch(
    std_srvs::srv::SetBool::Request::SharedPtr req,
    std_srvs::srv::SetBool::Response::SharedPtr res);

  /// A pointer to the GazeboROS node.
  gazebo_ros::Node::SharedPtr ros_node_;

  /// Publisher for gripper action status
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_;

  /// Service for gripper switch
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service_;

  /// Connection to event called at every world iteration.
  gazebo::event::ConnectionPtr update_connection_;

  /// Pointer to world.
  gazebo::physics::WorldPtr world_;

  /// Pointer to model.
  gazebo::physics::ModelPtr model_;

  /// Pointer to link.
  gazebo::physics::LinkPtr gripper_link_;

  /// Protect variables accessed on callbacks.
  std::mutex lock_;

  /// True if gripper is on.
  bool enabled_;
  bool model_attached_;

  /// Entities not affected by gripper.
  std::unordered_set<std::string> fixed_;

  /// Max distance to apply force.
  double max_distance_;

  /// List of models that the should pick up
  std::vector<std::string> parts_to_pick_;

  gazebo::physics::JointPtr picked_part_joint_;
};

GazeboRosVacuumGripper::GazeboRosVacuumGripper()
: impl_(std::make_unique<GazeboRosVacuumGripperPrivate>())
{
}

GazeboRosVacuumGripper::~GazeboRosVacuumGripper()
{
}

void GazeboRosVacuumGripper::Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  RCLCPP_WARN(rclcpp::get_logger("vacuum-gripper-plugin"), "GazeboRosVacuumGripper::Load entry");
  impl_->model_ = _model;
  impl_->world_ = _model->GetWorld();

  impl_->picked_part_joint_ = impl_->world_->Physics()->CreateJoint("fixed", impl_->model_);
  impl_->picked_part_joint_->SetName("picked_part_fixed_joints");

  // Initialize ROS node
  impl_->ros_node_ = gazebo_ros::Node::Get(_sdf);

  // Get QoS profiles
  const gazebo_ros::QoS & qos = impl_->ros_node_->get_qos();

    // Get gripper link
  if (_sdf->HasElement("link_name")) {
    auto link = _sdf->Get<std::string>("link_name");
    impl_->gripper_link_ = _model->GetLink(link);

    if (!impl_->gripper_link_) {
      RCLCPP_ERROR(rclcpp::get_logger("vacuum-gripper-plugin"), "Link [%s] not found. Aborting", link.c_str());
      impl_->ros_node_.reset();
      return;
    }
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("vacuum-gripper-plugin"), "Please specify <link_name>. Aborting.");
  }

  impl_->max_distance_ = 0.085;

  if (_sdf->HasElement("fixed")) {
    for (auto fixed = _sdf->GetElement("fixed"); fixed != nullptr;
      fixed = fixed->GetNextElement("fixed"))
    {
      auto name = fixed->Get<std::string>();
      impl_->fixed_.insert(name);
      RCLCPP_WARN(
        rclcpp::get_logger("vacuum-gripper-plugin"),
        "Model/Link [%s] exempted from gripper force", name.c_str());
    }
  }
  impl_->fixed_.insert(_model->GetName());
  impl_->fixed_.insert(impl_->gripper_link_->GetName());

   // Set list of models to pickup
  impl_->parts_to_pick_ = {"pump", "battery", "regulator", "sensor"};

  // Initialize publisher
  impl_->pub_ = impl_->ros_node_->create_publisher<std_msgs::msg::Bool>(
    "grasping", qos.get_publisher_qos("grasping", rclcpp::QoS(1)));

  RCLCPP_WARN(
    rclcpp::get_logger("vacuum-gripper-plugin"),
    "Advertise gripper status on [%s]", impl_->pub_->get_topic_name());

  // Initialize service
  impl_->service_ = impl_->ros_node_->create_service<std_srvs::srv::SetBool>(
    "switch",
    std::bind(
      &GazeboRosVacuumGripperPrivate::OnSwitch, impl_.get(),
      std::placeholders::_1, std::placeholders::_2));

  RCLCPP_WARN(
    rclcpp::get_logger("vacuum-gripper-plugin"),
    "Advertise gripper switch service on [%s]", impl_->service_->get_service_name());

  // Listen to the update event (broadcast every simulation iteration)
  impl_->update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
    std::bind(&GazeboRosVacuumGripperPrivate::OnUpdate, impl_.get()));

  RCLCPP_WARN(rclcpp::get_logger("vacuum-gripper-plugin"), "GazeboRosVacuumGripper::Load entry<exit>");
}

void GazeboRosVacuumGripperPrivate::OnUpdate()
{
#ifdef IGN_PROFILER_ENABLE
  IGN_PROFILE("GazeboRosVacuumGripper::OnUpdate");
#endif
  std_msgs::msg::Bool grasping_msg;

  if(enabled_ && !model_attached_){

    std::lock_guard<std::mutex> lock(lock_);

    ignition::math::Pose3d gripper_pose = gripper_link_->WorldPose();
    gazebo::physics::Model_V models = world_->Models();
    bool found = false;
    for (auto & model : models) {
        std::string model_name = model->GetName();
        std::string name = model_name.c_str();
        for(std::string part_type : parts_to_pick_){
          if (name.find(part_type) != std::string::npos) found = true;
        }
        if (!found) continue;

        if (fixed_.find(model_name) != fixed_.end()) {
            //RCLCPP_WARN(rclcpp::get_logger("vacuum-gripper-plugin"), "Model %s is fixed, skipping.", model_name.c_str());
            continue;
        }
        
        gazebo::physics::Link_V object_links = model->GetLinks();
        for (auto & object_link : object_links) {
            std::string link_name = object_link->GetName();
            ignition::math::Pose3d object_link_pose = object_link->WorldPose();
            ignition::math::Pose3d diff = gripper_pose - object_link_pose;
            
            if (diff.Pos().Length() > max_distance_) {
                continue;
            }
            picked_part_joint_->Load(gripper_link_, object_link, ignition::math::Pose3d());
            picked_part_joint_->Init();
            RCLCPP_WARN(rclcpp::get_logger("vacuum-gripper-plugin"), "Gripper attach");

            model_attached_ = true;
            grasping_msg.data = true;
        }
    }
  }
  else if(!enabled_ && model_attached_){
    picked_part_joint_->Detach();
    RCLCPP_WARN(rclcpp::get_logger("vacuum-gripper-plugin"), "Gripper de-attach");
    model_attached_ = false;
  }
  else if(enabled_ && model_attached_){
    grasping_msg.data = true;
  }
  pub_->publish(grasping_msg);
}

void GazeboRosVacuumGripperPrivate::OnSwitch(
  std_srvs::srv::SetBool::Request::SharedPtr req,
  std_srvs::srv::SetBool::Response::SharedPtr res)
{
  res->success = false;
  if (req->data) {
    if (!enabled_) {
      enabled_ = true;
      res->success = true;
      RCLCPP_WARN(rclcpp::get_logger("vacuum-gripper-plugin"), "Gripper on");
    } else {
      RCLCPP_WARN(rclcpp::get_logger("vacuum-gripper-plugin"), "Gripper is already on");
    }
  } else {
    if (enabled_) {
      enabled_ = false;
      res->success = true;
      RCLCPP_WARN(rclcpp::get_logger("vacuum-gripper-plugin"), "Gripper off");
    } else {
      RCLCPP_WARN(rclcpp::get_logger("vacuum-gripper-plugin"), "Gripper is already off");
    }
  }
}

GZ_REGISTER_MODEL_PLUGIN(GazeboRosVacuumGripper)
}  // namespace gazebo_plugins