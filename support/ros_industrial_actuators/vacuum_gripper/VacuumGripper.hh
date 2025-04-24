#ifndef _VACUUM_GRIPPER_PLUGIN_INCL_
#define _VACUUM_GRIPPER_PLUGIN_INCL_
#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/plugin/Register.hh>
#include <gz/msgs.hh>
#include <gz/transport/Node.hh>

#include <gz/sim/components/Pose.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Link.hh>
#include <gz/math/Pose3.hh>

#include <gz/sim/components/ParentEntity.hh>
//#include <gz/sim/components/ChildEntity.hh>
#include <gz/physics/Link.hh>

#include <gz/sim/components/Joint.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/DetachableJoint.hh>

#include <gz/sim/components/ChildLinkName.hh>

//#include <gz/sim/components/FixedJoint.hh>

#include <gz/msgs/boolean.pb.h>
//#include <gz/msgs/int32.pb.h>

#include <gz/sim/components/Name.hh>
#include <gz/sim/Entity.hh>


namespace vacuum_gripper
{

  class VacuumGripperPrivate;

  class VacuumGripper:
    // This class is a system.
    public gz::sim::System,
    // This class also implements the ISystemPostUpdate interface.
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPreUpdate
  {
    public: VacuumGripper();
 
    public: ~VacuumGripper();// override;

    public: void Configure(
      const gz::sim::Entity &_entity,
      const std::shared_ptr<const sdf::Element> &_sdf,
      gz::sim::EntityComponentManager &_ecm,
      gz::sim::EventManager &_eventMgr) override;

    public: void PreUpdate(
      const gz::sim::UpdateInfo &_info,
      gz::sim::EntityComponentManager &_ecm) override;

    private: std::unique_ptr<VacuumGripperPrivate> dataPtr;

    /// \brief Subscriber callbacks
    private:
      void OnEnableMessage(const gz::msgs::Boolean & msg);
      void CreatePublishers();
      void CreateSubscribers();
      void RemovePublishers();
      void RemoveSubscribers();
  };

}
#endif // _VACUUM_GRIPPER_PLUGIN_INCL_