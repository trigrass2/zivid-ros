#pragma once

#include "auto_generated_include_wrapper.h"

#include <Zivid/Settings.h>
#include <Zivid/Settings2D.h>

#include <ros/ros.h>

namespace Zivid
{
class Camera;
}

namespace zivid_camera
{
template <typename ConfigType>
class ConfigDRServer;

/// <summary>Controller that manages dynamic_reconfigure nodes for Settings and Settings2D</summary>
/// <remarks>
/// This is a templated class that handles Zivid::Settings and Zivid::Settings2D
/// configuration. It sets up the dynamic_reconfigure nodes (settings/, settings/acquisition_0 and so
/// on), handles callbacks and provides methods to convert from/to Zivid::Settings(2D) objects.
/// </remarks>
template <typename ZividSettingsType, typename SettingsConfigType, typename SettingsAcquisitionConfigType>
class CaptureSettingsController
{
  static_assert(std::is_same_v<ZividSettingsType, Zivid::Settings> ||
                std::is_same_v<ZividSettingsType, Zivid::Settings2D>);
  static_assert(std::is_same_v<SettingsConfigType, SettingsConfig> ||
                std::is_same_v<SettingsConfigType, Settings2DConfig>);
  static_assert(std::is_same_v<SettingsAcquisitionConfigType, SettingsAcquisitionConfig> ||
                std::is_same_v<SettingsAcquisitionConfigType, Settings2DAcquisitionConfig>);

public:
  CaptureSettingsController(ros::NodeHandle& nh, Zivid::Camera& camera, const std::string& node_name,
                            std::size_t num_acquisition_servers);
  ~CaptureSettingsController();
  ZividSettingsType zividSettings() const;
  void setZividSettings(const ZividSettingsType& settings);
  std::size_t numAcquisitionConfigServers() const
  {
    return acquisition_config_dr_servers_.size();
  }

private:
  using SettingsConfigTypeDRServer = ConfigDRServer<SettingsConfigType>;
  using SettingsAcquisitionConfigTypeDRServer = ConfigDRServer<SettingsAcquisitionConfigType>;
  std::string node_name_;
  std::unique_ptr<SettingsConfigTypeDRServer> general_config_dr_server_;
  std::vector<std::unique_ptr<SettingsAcquisitionConfigTypeDRServer>> acquisition_config_dr_servers_;
};

};  // namespace zivid_camera
