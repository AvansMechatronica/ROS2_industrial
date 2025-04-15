#include "VacuumGripper.hh"


#include <iostream>
#include <vector>

using namespace vacuum_gripper;

class vacuum_gripper::VacuumGripperPrivate
{
  public:
    gz::transport::Node node_;
    gz::transport::Node::Publisher status_pub_;

    std::string namespace_ = "";
    std::string enable_topic_ = "/vacuum_gripper_control";
    std::string status_topic_ = "/vacuum_gripper_status";

    bool status = false;
    bool enabled = false;  

  /// True if gripper is on.
  bool enabled_;
  bool model_attached_;

  /// Max distance to apply force.
  double max_distance_;

  /// List of models that the should pick up
  std::vector<std::string> parts_to_pick_;

  /// Protect variables accessed on callbacks.
  std::mutex lock_;

};

VacuumGripper::VacuumGripper(): dataPtr(new VacuumGripperPrivate())
{
  gzmsg << "VacuumGripper::VacuumGripper" << std::endl;

  CreatePublishers();
  CreateSubscribers();


  // Set list of models to pickup
  dataPtr->parts_to_pick_ = {"pump", "battery", "regulator", "sensor"};

  dataPtr->max_distance_ = 0.085;
}
 
VacuumGripper::~VacuumGripper()
{
  gzmsg << "VacuumGripper::~VacuumGripper" << std::endl;
  RemovePublishers();
  RemoveSubscribers();
  dataPtr.reset();
}

void VacuumGripper::CreatePublishers()
{
  gzmsg << "VacuumGripper::CreatePublishers" << std::endl;
  dataPtr->status_pub_ = gz::transport::Node::Publisher();
  dataPtr->status_pub_ = dataPtr->node_.Advertise < gz::msgs::Boolean> (dataPtr->status_topic_);
}

void VacuumGripper::CreateSubscribers()
{
  gzmsg << "VacuumGripper::CreateSubscribers" << std::endl;
  dataPtr->node_.Subscribe(dataPtr->enable_topic_, &VacuumGripper::OnEnableMessage, this);
}

void VacuumGripper::RemovePublishers()
{
  dataPtr->node_.UnadvertiseSrv(dataPtr->status_topic_);
}


void VacuumGripper::RemoveSubscribers()
{
  dataPtr->node_.Unsubscribe(dataPtr->enable_topic_);
}


void VacuumGripper::OnEnableMessage(const gz::msgs::Boolean & msg){
  gzmsg << "VacuumGripper::OnEnableMessage" << std::endl;
  dataPtr->enabled = msg.data();

  if (dataPtr->enabled) {
    if (!dataPtr->enabled_) {
      dataPtr->enabled_ = true;
      gzmsg << "Gripper on"<< std::endl;
    } else {
      gzmsg << "Gripper is already on" << std::endl;
    }
  } else {
    if (dataPtr->enabled_) {
      dataPtr->enabled_ = false;
      gzmsg << "Gripper off"<< std::endl;
    } else {
      gzmsg << "Gripper is already off" << std::endl;
    }
  }

}

void VacuumGripper::Configure(
  const gz::sim::Entity &_entity,
  const std::shared_ptr<const sdf::Element> &_sdf,
  const gz::sim::EntityComponentManager &_ecm,
  const gz::sim::EventManager &_eventMgr){
    gzmsg << "VacuumGripper::Configure" << std::endl;
  }
 
void VacuumGripper::PostUpdate(const gz::sim::UpdateInfo &_info,
    const gz::sim::EntityComponentManager &_ecm)
{
  //gzmsg << "VacuumGripper::PostUpdate" << std::endl;
  // Check if the gripper is enabled

  if(dataPtr->enabled)
  {
    dataPtr->status = !dataPtr->status; // just testing
  }


  if(dataPtr->enabled_ && !dataPtr->model_attached_){
    gzmsg << "Gripper attach" << std::endl;
    dataPtr->model_attached_ = true;
    dataPtr->status = true;
  }
  else if(!dataPtr->enabled_ && dataPtr->model_attached_){
    gzmsg << "Gripper de-attach" << std::endl;
    dataPtr->status = false;
  }
  else if(dataPtr->enabled_ && dataPtr->model_attached_){

    dataPtr->status = true;
  }
  gz::msgs::Boolean status_msg;

  status_msg.set_data(dataPtr->status);

  dataPtr->status_pub_.Publish(status_msg);

}

void VacuumGripper::Update(const gz::sim::UpdateInfo &_info,
  const gz::sim::EntityComponentManager &_ecm)
{
  gzmsg << "VacuumGripper::Update" << std::endl;


  // Check if the gripper is enabled

  if(dataPtr->enabled)
  {
    dataPtr->status = !dataPtr->status; // just testing
  }

  gz::msgs::Boolean status_msg;

  status_msg.set_data(dataPtr->status);

  dataPtr->status_pub_.Publish(status_msg);

}

// Include a line in your source file for each interface implemented.
GZ_ADD_PLUGIN(
  vacuum_gripper::VacuumGripper,
  gz::sim::System,
  //vacuum_gripper::VacuumGripper::ISystemConfigure//,
  vacuum_gripper::VacuumGripper::ISystemPostUpdate//,
  //vacuum_gripper::VacuumGripper::ISystemUpdate
)
