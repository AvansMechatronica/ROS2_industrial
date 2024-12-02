#ifndef CUSTOM_LOGICAL_CAMERA_PLUGIN_HPP_
#define CUSTOM_LOGICAL_CAMERA_PLUGIN_HPP_

#include <gazebo/common/Plugin.hh>
#include <ros_industrial_msgs/msg/sensors.hpp>

#include <memory>

namespace ros_industrial_sensors
{

class CustomLogicalCameraPluginPrivate;

/// Plugin to attach to a gazebo CustomLogicalCameraPlugin sensor and publish ROS message of output
class CustomLogicalCameraPlugin : public gazebo::SensorPlugin
{
public:
  /// Constructor.
  CustomLogicalCameraPlugin();
  /// Destructor.
  virtual ~CustomLogicalCameraPlugin();

  // Documentation Inherited
  void Load(gazebo::sensors::SensorPtr _sensor, sdf::ElementPtr _sdf) override;


private:
  /// Private data pointer
  std::unique_ptr<CustomLogicalCameraPluginPrivate> impl_;
};

}  // namespace ariac_sensors

#endif  // CUSTOM_LOGICAL_CAMERA_PLUGIN_HPP_
