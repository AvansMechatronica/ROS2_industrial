#include <gz/sim/System.hh>
 
namespace vacuum_gripper
{
  class VacuumGripper:
    // This class is a system.
    public gz::sim::System,
    // This class also implements the ISystemPostUpdate interface.
    public gz::sim::ISystemPostUpdate
  {
    public: VacuumGripper();
 
    public: ~VacuumGripper() override;
 
    public: void PostUpdate(const gz::sim::UpdateInfo &_info,
                const gz::sim::EntityComponentManager &_ecm) override;
  };
 
  class VacuumGripper2:
    // This class is a system.
    public gz::sim::System,
    // This class also implements the ISystemPreUpdate, ISystemUpdate,
    // and ISystemPostUpdate interfaces.
    public gz::sim::ISystemPreUpdate,
    public gz::sim::ISystemUpdate,
    public gz::sim::ISystemPostUpdate,
    public gz::sim::ISystemReset
  {
    public: VacuumGripper2();
 
    public: ~VacuumGripper2() override;
 
    public: void PreUpdate(const gz::sim::UpdateInfo &_info,
                gz::sim::EntityComponentManager &_ecm) override;
 
    public: void Update(const gz::sim::UpdateInfo &_info,
                gz::sim::EntityComponentManager &_ecm) override;
 
    public: void PostUpdate(const gz::sim::UpdateInfo &_info,
                const gz::sim::EntityComponentManager &_ecm) override;
 
    public: void Reset(const gz::sim::UpdateInfo &_info,
                 gz::sim::EntityComponentManager &_ecm) override;
  };
}