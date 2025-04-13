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
  
};

VacuumGripper::VacuumGripper(): dataPtr(new VacuumGripperPrivate())
{
  CreatePublishers();
  CreateSubscribers();
}
 
VacuumGripper::~VacuumGripper()
{
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

}
 
void VacuumGripper::PostUpdate(const gz::sim::UpdateInfo &_info,
    const gz::sim::EntityComponentManager &_ecm)
{
  gzmsg << "VacuumGripper::PostUpdate" << std::endl;
}

#if 0
void VacuumGripper::Update(const gz::sim::UpdateInfo &_info,
  const gz::sim::EntityComponentManager &_ecm)
{
gzmsg << "VacuumGripper::Update" << std::endl;
}
#endif

// Include a line in your source file for each interface implemented.
GZ_ADD_PLUGIN(
  vacuum_gripper::VacuumGripper,
  gz::sim::System,
  vacuum_gripper::VacuumGripper::ISystemPostUpdate//,
  //vacuum_gripper::VacuumGripper::ISystemUpdate
)
