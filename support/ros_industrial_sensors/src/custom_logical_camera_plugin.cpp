#include <ros_industrial_sensors/custom_logical_camera_plugin.hpp>

#include <gazebo/sensors/LogicalCameraSensor.hh>

#include <gazebo_ros/conversions/builtin_interfaces.hpp>
#include <gazebo_ros/conversions/geometry_msgs.hpp>
#include <gazebo_ros/node.hpp>
#include <gazebo_ros/utils.hpp>

#include <ros_industrial_msgs/msg/logical_camera_image.hpp>
#include <ros_industrial_msgs/msg/part_pose.hpp>

#include <memory>

namespace ros_industrial_sensors
{

class CustomLogicalCameraPluginPrivate
{
public:
  /// Node for ros communication
  gazebo_ros::Node::SharedPtr ros_node_;

  /// Publish for logical camera message
  rclcpp::Publisher<ros_industrial_msgs::msg::LogicalCameraImage>::SharedPtr basic_pub_;

  /// LogicalCameraImage message modified each update
  ros_industrial_msgs::msg::LogicalCameraImage::SharedPtr basic_image_msg_;
  
  /// CustomLogicalCameraPlugin sensor this plugin is attached to
  gazebo::sensors::LogicalCameraSensorPtr sensor_;
  
  /// Event triggered when sensor updates
  gazebo::event::ConnectionPtr sensor_update_event_;
  
  /// List of models that the logical camera will publish
  std::vector<std::string> parts_to_publish_;

  std::string camera_name_;
  std::string sensor_type_;
  std::string camera_frame_name_;

  std::map<std::string, int> part_types_;

  /// Publish latest logical camera data to ROS
  void OnUpdate();
};

CustomLogicalCameraPlugin::CustomLogicalCameraPlugin()
: impl_(std::make_unique<CustomLogicalCameraPluginPrivate>())
{
}

CustomLogicalCameraPlugin::~CustomLogicalCameraPlugin()
{
}

void CustomLogicalCameraPlugin::Load(gazebo::sensors::SensorPtr _sensor, sdf::ElementPtr _sdf)
{
  RCLCPP_INFO(rclcpp::get_logger("logical camera plugin"), "CustomLogicalCameraPlugin::Load entry");

  impl_->sensor_ = std::dynamic_pointer_cast<gazebo::sensors::LogicalCameraSensor>(_sensor);
  impl_->ros_node_ = gazebo_ros::Node::Get(_sdf);

   // Set list of models to publish
  impl_->parts_to_publish_ = {"pump", "battery", "regulator", "sensor"};

  impl_->camera_name_ = _sdf->Get<std::string>("camera_name");
  impl_->camera_frame_name_ = _sdf->Get<std::string>("camera_frame_name");

  impl_->basic_pub_ = impl_->ros_node_->create_publisher<ros_industrial_msgs::msg::LogicalCameraImage>(
    "ros_industrial/sensors/" + impl_->camera_name_ + "/image", rclcpp::SensorDataQoS());

  impl_->basic_image_msg_ = std::make_shared<ros_industrial_msgs::msg::LogicalCameraImage>();

  impl_->sensor_update_event_ = impl_->sensor_->ConnectUpdated(
    std::bind(&CustomLogicalCameraPluginPrivate::OnUpdate, impl_.get()));

  //RCLCPP_INFO(rclcpp::get_logger("logical camera plugin"), "CustomLogicalCameraPlugin::Load exit");
}

void CustomLogicalCameraPluginPrivate::OnUpdate()
{
  //RCLCPP_INFO(rclcpp::get_logger("logical camera plugin"), "CustomLogicalCameraPluginPrivate::OnUpdate entry");

  const auto & image = this->sensor_->Image();

  geometry_msgs::msg::Pose sensor_pose = gazebo_ros::Convert<geometry_msgs::msg::Pose>(
    gazebo::msgs::ConvertIgn(image.pose()));

  std::vector<ros_industrial_msgs::msg::PartPose> part_poses;

  for (int i = 0; i < image.model_size(); i++) {

    const auto & lc_model = image.model(i);
    std::string name = lc_model.name();


    for(std::string part_type : parts_to_publish_){
      //std::cout << "part_type " << part_type << std::endl;
      if (name.find(part_type) != std::string::npos) {
        //RCLCPP_INFO(rclcpp::get_logger("logical camera plugin"), "part found");
        ros_industrial_msgs::msg::PartPose part_pose;

        //part.part.type = part_types_[part_type];
        part_pose.part.data = name;
        part_pose.pose = gazebo_ros::Convert<geometry_msgs::msg::Pose>(gazebo::msgs::ConvertIgn(lc_model.pose()));

        part_poses.push_back(part_pose);

        break;
      }
    }
  }

  basic_image_msg_->camera_frame.data = camera_frame_name_;

  basic_image_msg_->part_poses.clear();

  for (ros_industrial_msgs::msg::PartPose &part_pose : part_poses) {
    basic_image_msg_->part_poses.push_back(part_pose);
  }

  basic_pub_->publish(*basic_image_msg_);
}

GZ_REGISTER_SENSOR_PLUGIN(CustomLogicalCameraPlugin)

}  // namespace ros_industrial_sensors
