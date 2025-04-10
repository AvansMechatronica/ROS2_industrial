#include "VacuumGripper.hh"

#include <gz/msgs/int32.pb.h>

#include <iostream>
#include <vector>

#include <gz/plugin/Register.hh>


using namespace vacuum_gripper;
 
VacuumGripper::VacuumGripper()
{
}
 
VacuumGripper::~VacuumGripper()
{
}
 
void VacuumGripper::PostUpdate(const gz::sim::UpdateInfo &_info,
    const gz::sim::EntityComponentManager &_ecm)
{
  gzmsg << "VacuumGripper::PostUpdate" << std::endl;
}

// Include a line in your source file for each interface implemented.
GZ_ADD_PLUGIN(
  vacuum_gripper::VacuumGripper,
  gz::sim::System,
  vacuum_gripper::VacuumGripper::ISystemPostUpdate)
