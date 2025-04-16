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
    std::string enable_topic_ = "/vacuum_gripper/control/enable";
    std::string status_topic_ = "/vacuum_gripper/status/attached";

    bool status = false;
    bool enabled = false;  

    /// True if gripper is on.
    bool enabled_;
    bool model_attached_;

    /// Max distance to apply force.
    double max_distance_;

    std::string link_name;

    /// List of models that the should pick up
    std::vector<std::string> parts_to_pick_;

    /// Pointer to link.
    //gz::physics::LinkPtr gripper_link_;

    /// Protect variables accessed on callbacks.
    std::mutex lock_;

    bool once = false;
};

VacuumGripper::VacuumGripper(): dataPtr(new VacuumGripperPrivate())
{
  gzmsg << "VacuumGripper: VacuumGripper()" << std::endl;

  CreatePublishers();
  CreateSubscribers();


  // Set list of models to pickup
  dataPtr->parts_to_pick_ = {"pump", "battery", "regulator", "sensor"};

  dataPtr->max_distance_ = 0.085;
}
 
VacuumGripper::~VacuumGripper()
{
  gzmsg << "VacuumGripper: ~VacuumGripper()" << std::endl;
  RemovePublishers();
  RemoveSubscribers();
  dataPtr.reset();
}

void VacuumGripper::CreatePublishers()
{
  gzmsg << "VacuumGripper: CreatePublishers()" << std::endl;
  dataPtr->status_pub_ = gz::transport::Node::Publisher();
  dataPtr->status_pub_ = dataPtr->node_.Advertise < gz::msgs::Boolean> (dataPtr->status_topic_);
}

void VacuumGripper::CreateSubscribers()
{
  gzmsg << "VacuumGripper: CreateSubscribers()" << std::endl;
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
  gzmsg << "VacuumGripper: OnEnableMessage()" << std::endl;
  dataPtr->enabled = msg.data();

  if (dataPtr->enabled) {
    if (!dataPtr->enabled_) {
      dataPtr->enabled_ = true;
      gzmsg << "VacuumGripper: Gripper on"<< std::endl;
    } else {
      gzmsg << "VacuumGripper: Gripper is already on" << std::endl;
    }
  } else {
    if (dataPtr->enabled_) {
      dataPtr->enabled_ = false;
      gzmsg << "VacuumGripper: Gripper off"<< std::endl;
    } else {
      gzmsg << "VacuumGripper: Gripper is already off" << std::endl;
    }
  }

}

void VacuumGripper::Configure(
  const gz::sim::Entity &_entity,
  const std::shared_ptr<const sdf::Element> &_sdf,
  gz::sim::EntityComponentManager &_ecm,
  gz::sim::EventManager &_eventMgr)
  {
    gzmsg << "VacuumGripper: Configure()" << std::endl;
    if (_sdf->HasElement("link_name")) {
      dataPtr->link_name = _sdf->Get<std::string>("link_name");
      //dataPtr->link_name = "link1";
      gzmsg << "VacuumGripper: Link name found " << dataPtr->link_name << std::endl;
      //impl_->gripper_link_ = _model->GetLink(link);
    }
    else{
      gzerr << "VacuumGripper: No link defined)" << std::endl;

    }
  }
 
void VacuumGripper::Update(const gz::sim::UpdateInfo &_info,
    gz::sim::EntityComponentManager &_ecm)
{
  //gzmsg << "VacuumGripper: Update()" << std::endl;
  // Check if the gripper is enabled

  gz::math::Pose3d gripper_pose;
  std::optional<gz::sim::Entity> foundEntity;

  if(!dataPtr->once){
    //dataPtr->once = true;
  _ecm.Each<gz::sim::components::Name, gz::sim::components::Pose>(
    [&](const gz::sim::Entity &_entity,
        const gz::sim::components::Name *_nameComp,
        const gz::sim::components::Pose *_poseComp) -> bool
    {
      //if (!_nameComp) return true; // Continue iterating
      //if (_nameComp && (_nameComp->Data() == dataPtr->link_name))
      if (_nameComp)
      {
        //gzmsg << "Found entity with name: " << dataPtr->link_name << ", ID: " << _entity << std::endl;
        gzmsg << "Found entity with name: " << _nameComp->Data() << ", ID: " << _entity << std::endl;
        foundEntity = _entity;
        gripper_pose = _poseComp->Data();
        gzmsg << "Gripper pose: " << gripper_pose << std::endl;

        //return false; // Stop iterating once the entity is found
      }
      //gzerr << "No found entity with name: " << dataPtr->link_name << std::endl;
      return true; // Continue iterating
    });
  }

#if 0
  // Iterate through all entities with a specific component (e.g., Name or Pose)
  _ecm.Each<gz::sim::components::Name, gz::sim::components::Pose>(
    [&](const gz::sim::Entity &_entity,
        const gz::sim::components::Name *_nameComp,
        const gz::sim::components::Pose *_poseComp) -> bool
    {
      if (!_nameComp || !_poseComp)
      {
        gzerr << "Entity " << _entity << " is missing required components." << std::endl;
        return true; // Continue iterating
      }
  
      // Access the entity's name
      std::string entityName = _nameComp->Data();
      if(entityName != dataPtr->link_name){ // Skip gripper link
  
        // Access the entity's pose
        gz::math::Pose3d object_pose = _poseComp->Data();

        gz::math::Pose3d diff = gripper_pose - object_pose;
            
        if (diff.Pos().Length() < dataPtr->max_distance_) {
          gzmsg << "Entity ID: " << _entity << ", Name: " << entityName << std::endl;
          gzmsg << "Object pose: " << object_pose << std::endl;
          gzmsg << "Gripper pose: " << gripper_pose << std::endl;
          gzmsg << "Distance : " << diff.Pos().Length()  << std::endl;
          //continue;
        }

      }

  
      return true; // Continue iterating
    });
#endif

  if(dataPtr->enabled)
  {
    dataPtr->status = !dataPtr->status; // just testing
  }


  if(dataPtr->enabled_ && !dataPtr->model_attached_){
    gzmsg << "VacuumGripper: Gripper attach" << std::endl;
    dataPtr->model_attached_ = true;
    dataPtr->status = true;
  }
  else if(!dataPtr->enabled_ && dataPtr->model_attached_){
    gzmsg << "VacuumGripper: Gripper de-attach()" << std::endl;
    dataPtr->status = false;
  }
  else if(dataPtr->enabled_ && dataPtr->model_attached_){

    dataPtr->status = true;
  }
  gz::msgs::Boolean status_msg;


  dataPtr->status = dataPtr->status? false : true;
  status_msg.set_data(dataPtr->status);

//  dataPtr->status_pub_.Publish(status_msg);

  if (!dataPtr->status_pub_.Publish(status_msg)) {
    gzerr << "gz::msgs::Int32 message couldn't be published at topic: " <<
    dataPtr->status_topic_ << std::endl;
  }

}


// Include a line in your source file for each interface implemented.
GZ_ADD_PLUGIN(
  vacuum_gripper::VacuumGripper,
  gz::sim::System,
  vacuum_gripper::VacuumGripper::ISystemConfigure,
  vacuum_gripper::VacuumGripper::ISystemUpdate
)
