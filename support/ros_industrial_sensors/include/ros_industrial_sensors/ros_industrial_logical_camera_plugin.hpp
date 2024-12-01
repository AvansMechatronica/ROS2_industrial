#ifndef ROS_INDUSTRAIL_LOGICAL_CAMERA_PLUGIN_HPP_
#define ROS_INDUSTRAIL_LOGICAL_CAMERA_PLUGIN_HPP_

#include <gazebo/common/Plugin.hh>
#include <ros_industrial_msgs/msg/sensors.hpp>

#include <memory>

namespace ROS_INDUSTRAIL_sensors
{

class RosIndustrialLogicalCameraPluginPrivate;

/// Plugin to attach to a gazebo RosIndustrialLogicalCameraPlugin sensor and publish ROS message of output
class RosIndustrialLogicalCameraPlugin : public gazebo::SensorPlugin
{
public:
  /// Constructor.
  RosIndustrialLogicalCameraPlugin();
  /// Destructor.
  virtual ~RosIndustrialLogicalCameraPlugin();

  // Documentation Inherited
  void Load(gazebo::sensors::SensorPtr _sensor, sdf::ElementPtr _sdf) override;

  void SensorHealthCallback(const ros_industrial_msgs::msg::Sensors::SharedPtr msg);

private:
  /// Private data pointer
  std::unique_ptr<RosIndustrialLogicalCameraPluginPrivate> impl_;
};

}  // namespace ROS_INDUSTRAIL_sensors

#endif  // ROS_INDUSTRAIL_LOGICAL_CAMERA_PLUGIN_HPP_
