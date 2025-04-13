#ifndef _VACUUM_GRIPPER_PLUGIN_INCL_
#define _VACUUM_GRIPPER_PLUGIN_INCL_
#include <gz/sim/System.hh>
#include <gz/plugin/Register.hh>
#include <gz/msgs.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs/boolean.pb.h>


namespace vacuum_gripper
{

  class VacuumGripperPrivate;

  class VacuumGripper:
    // This class is a system.
    public gz::sim::System,
    // This class also implements the ISystemPostUpdate interface.
    public gz::sim::ISystemPostUpdate//,
//    public gz::sim::ISystemUpdate
  {
    public: VacuumGripper();
 
    public: ~VacuumGripper() override;
 
    public: void PostUpdate(const gz::sim::UpdateInfo &_info,
                const gz::sim::EntityComponentManager &_ecm) override;
#if 0
    public: void Update(const gz::sim::UpdateInfo &_info,
                  const gz::sim::EntityComponentManager &_ecm) override;
#endif
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