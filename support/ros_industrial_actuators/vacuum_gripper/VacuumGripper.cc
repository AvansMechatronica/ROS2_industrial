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
};

VacuumGripper::VacuumGripper(): dataPtr(new VacuumGripperPrivate())
{
  gzmsg << "VacuumGripper::VacuumGripper" << std::endl;

  CreatePublishers();
  CreateSubscribers();
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
}

void VacuumGripper::Configure(
  const Entity &_entity,
  const std::shared_ptr<const sdf::Element> &_sdf,
  EntityComponentManager &_ecm,
  EventManager &_eventMgr){
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
  vacuum_gripper::VacuumGripper::ISystemPostUpdate//,
  //vacuum_gripper::VacuumGripper::ISystemUpdate
)
