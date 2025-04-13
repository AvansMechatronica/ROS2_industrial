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
    std::string enable_topic_ = "/enable";
    std::string status_topic_ = "/status";

    bool status = false;
    bool enabled = false;  
};

VacuumGripper::VacuumGripper(): dataPtr(new VacuumGripperPrivate())
{
  CreatePublishers();
  CreateSubscribers();
}
 
VacuumGripper::~VacuumGripper()
{
  RemovePublishers();
  RemoveSubscribers();
  dataPtr.reset();
}

void VacuumGripper::CreatePublishers()
{
  dataPtr->status_pub_ = gz::transport::Node::Publisher();
  dataPtr->status_pub_ = dataPtr->node_.Advertise < gz::msgs::Boolean> (dataPtr->status_topic_);
}

void VacuumGripper::CreateSubscribers()
{
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
  dataPtr->enabled = msg.data();
}
 
void VacuumGripper::PostUpdate(const gz::sim::UpdateInfo &_info,
    const gz::sim::EntityComponentManager &_ecm)
{
  gzmsg << "VacuumGripper::PostUpdate" << std::endl;
}

void VacuumGripper::PreUpdate(const gz::sim::UpdateInfo &_info,
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
