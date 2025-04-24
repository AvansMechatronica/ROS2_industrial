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

    //bool status = false;
    //bool enabled = false;  

    /// True if gripper is on.
    bool gripper_enabled;
    bool model_attached;

    /// Max distance to apply force.
    double max_distance_;

    gz::sim::Model model;
    std::string link_name;

    gz::sim::Entity detachableJointEntity{gz::sim::kNullEntity};

    /// List of models that the should pick up
    //std::vector<std::string> parts_to_pick_;

    /// Pointer to link.
    //gz::physics::LinkPtr gripper_link_;

    /// Protect variables accessed on callbacks.
    //std::mutex lock_;

    /// Pointer to joint.

    // gz::sim::Entity jointEntity = gz::sim::kNullEntity;
};

VacuumGripper::VacuumGripper(): dataPtr(new VacuumGripperPrivate())
{
  gzmsg << "VacuumGripper: VacuumGripper()" << std::endl;

  CreatePublishers();
  CreateSubscribers();


  // Set list of models to pickup
  //dataPtr->parts_to_pick_ = {"pump", "battery", "regulator", "sensor"};

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
  bool enabled = msg.data();

  if (enabled) {
    if (!dataPtr->gripper_enabled) {
      dataPtr->gripper_enabled = true;
      gzmsg << "VacuumGripper: Gripper on"<< std::endl;
    } else {
      gzmsg << "VacuumGripper: Gripper is already on" << std::endl;
    }
  } else {
    if (dataPtr->gripper_enabled) {
      dataPtr->gripper_enabled = false;
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

  dataPtr->model = gz::sim::Model(_entity);
  if (!dataPtr->model.Valid(_ecm))
  {
    gzerr << "DetachableJoint should be attached to a model entity. "
           << "Failed to initialize." << std::endl;
    return;
  }
  if (_sdf->HasElement("link_name")) {
    dataPtr->link_name = _sdf->Get<std::string>("link_name");
    //dataPtr->link_name = "link1";
    gzmsg << "VacuumGripper: Link name found " << dataPtr->link_name << std::endl;
    //dataPtr->gripper_link_ = _model->GetLink(dataPtr->link_name);

  }
  else{
    gzerr << "VacuumGripper: No link defined)" << std::endl;
  }
}
 
void VacuumGripper::PreUpdate(
  const gz::sim::UpdateInfo &_info,
  gz::sim::EntityComponentManager &_ecm)
{
  gz::math::Pose3d gripper_pose;
  std::optional<gz::sim::Entity> gripper_entity;

  // Check if the gripper is enabled and no model is currently attached
  if (dataPtr->gripper_enabled && !dataPtr->model_attached)
  {
    // Find the gripper entity and its pose
    _ecm.Each<gz::sim::components::Name, gz::sim::components::Pose>(
      [&](const gz::sim::Entity &_entity,
          const gz::sim::components::Name *_nameComp,
          const gz::sim::components::Pose *_poseComp) -> bool
      {
        if (_nameComp && (_nameComp->Data() == dataPtr->link_name))
        {
          gzmsg << "Found gripper entity with name: " << _nameComp->Data() << ", ID: " << _entity << std::endl;
          gripper_entity = _entity;
          gripper_pose = _poseComp->Data();
          gzmsg << "Gripper pose: " << gripper_pose << std::endl;
          return false; // Stop iterating once the gripper is found
          if(!_ecm.CreateComponent(_entity, gz::sim::components::Joint()))
          {
            gzerr << "Failed to create joint component for entity: " << _entity << std::endl;
            return true; // Continue iterating
          }
              }
        return true; // Continue iterating
      });

    if (!gripper_entity)
    {
      //gzerr << "VacuumGripper: Gripper entity not found." << std::endl;
      return;
    }

    // Find objects within range of the gripper
    _ecm.Each<gz::sim::components::Name, gz::sim::components::Pose>(
      [&](const gz::sim::Entity &object_entity,
          const gz::sim::components::Name *_nameComp,
          const gz::sim::components::Pose *_poseComp) -> bool
      {
        if (!_nameComp || !_poseComp)
        {
          gzerr << "Entity " << object_entity << " is missing required components." << std::endl;
          return true; // Continue iterating
        }

        std::string entityName = _nameComp->Data();
        if (entityName != dataPtr->link_name) // Skip the gripper link itself
        {
          gz::math::Pose3d object_pose = _poseComp->Data();
          gz::math::Pose3d diff = gripper_pose - object_pose;

          if (diff.Pos().Length() < dataPtr->max_distance_)
          {
            gzmsg << "Entity within range: " << entityName << ", ID: " << object_entity << std::endl;
            gzmsg << "Object pose: " << object_pose << std::endl;
            gzmsg << "Distance: " << diff.Pos().Length() << std::endl;

            // Attach object to gripper here
            {


              auto objectLinkEntity = _ecm.EntityByComponents(
                gz::sim::components::Link(), gz::sim::components::ParentEntity(object_entity));//,

              if (!objectLinkEntity){
                gzerr << "Failed to find object link entity for: " << entityName << std::endl;
                return false; // Stop iterating

              }
              gzerr << "Object link entity: " << objectLinkEntity << std::endl;

              dataPtr->detachableJointEntity = _ecm.CreateEntity();

              auto component = _ecm.CreateComponent(
                dataPtr->detachableJointEntity,
                gz::sim::components::DetachableJoint({gripper_entity.value(),
                  objectLinkEntity, "fixed"}));
              if (!component)
              {
  	            gzmsg << "Failed to create DetachableJoint component for entity: " << dataPtr->detachableJointEntity << std::endl;
                return true; // Continue iterating
              }
              //gzmsg << "connect entity " << object_entity << " to entty " << gripper_entity.value() << std::endl;

            }

            gzmsg << "VacuumGripper: Object attached to gripper." << std::endl;

            dataPtr->model_attached = true;
            return false; // Stop iterating once an object is attached
          }
        }
        return true; // Continue iterating
      });
  }
  else if (!dataPtr->gripper_enabled && dataPtr->model_attached)
  {
    gzmsg << "VacuumGripper: Detaching object from gripper." << std::endl;
    dataPtr->model_attached = false;

    // Remove the joint entity
    #if 1
    if (dataPtr->detachableJointEntity != gz::sim::kNullEntity)
    {
      _ecm.RequestRemoveEntity(dataPtr->detachableJointEntity);
      dataPtr->detachableJointEntity = gz::sim::kNullEntity;
    }
    #endif
  }

  // Publish the gripper status
  gz::msgs::Boolean status_msg;
  status_msg.set_data(dataPtr->model_attached);

  if (!dataPtr->status_pub_.Publish(status_msg))
  {
    gzerr << "VacuumGripper: Failed to publish status message on topic: " << dataPtr->status_topic_ << std::endl;
  }
}


// Include a line in your source file for each interface implemented.
GZ_ADD_PLUGIN(
  vacuum_gripper::VacuumGripper,
  gz::sim::System,
  vacuum_gripper::VacuumGripper::ISystemConfigure,
  vacuum_gripper::VacuumGripper::ISystemPreUpdate
)