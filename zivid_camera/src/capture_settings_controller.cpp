#include "capture_settings_controller.h"

#include "SettingsConfigUtils.h"
#include "SettingsAcquisitionConfigUtils.h"
#include "Settings2DConfigUtils.h"
#include "Settings2DAcquisitionConfigUtils.h"

#include <Zivid/Camera.h>
#include <Zivid/Experimental/SettingsInfo.h>

#include <dynamic_reconfigure/server.h>

namespace zivid_camera
{
template <typename ConfigType>
class ConfigDRServer
{
public:
  template <typename ZividSettings>
  ConfigDRServer(const std::string& name, ros::NodeHandle& nh, const ZividSettings& defaultSettings)
    : name_(name), dr_server_(dr_server_mutex_, ros::NodeHandle(nh, name_)), config_(ConfigType::__getDefault__())
  {
    static_assert(std::is_same_v<ZividSettings, Zivid::Settings> || std::is_same_v<ZividSettings, Zivid::Settings2D> ||
                  std::is_same_v<ZividSettings, Zivid::Settings::Acquisition> ||
                  std::is_same_v<ZividSettings, Zivid::Settings2D::Acquisition>);

    const auto config_min = zividSettingsToMinConfig<ConfigType>(defaultSettings);
    dr_server_.setConfigMin(config_min);

    const auto config_max = zividSettingsToMaxConfig<ConfigType>(defaultSettings);
    dr_server_.setConfigMax(config_max);

    const auto default_config = zividSettingsToConfig<ConfigType>(defaultSettings);
    dr_server_.setConfigDefault(default_config);

    setConfig(default_config);

    auto cb = [this](const ConfigType& config, uint32_t /*level*/) {
      ROS_INFO("Configuration '%s' changed", name_.c_str());
      config_ = config;
      if constexpr (std::is_same_v<ConfigType, SettingsAcquisitionConfig>)
      {
        ROS_INFO("Updating config for %s", name_.c_str());
        ROS_INFO("config_.enabled=%d", config_.enabled);
        ROS_INFO("config_.aperture=%.6f", config_.aperture);
        ROS_INFO("config_.exposure_time=%ul", config_.exposure_time);
        ROS_INFO("config_.gain=%.6f", config_.gain);
        ROS_INFO("config_.brightness=%.6f", config_.brightness);
      }
    };
    using CallbackType = typename decltype(dr_server_)::CallbackType;
    dr_server_.setCallback(CallbackType(cb));
  }

  void setConfig(const ConfigType& cfg)
  {
    config_ = cfg;
    dr_server_.updateConfig(config_);
  }

  const ConfigType& config() const
  {
    return config_;
  }

  const std::string& name() const
  {
    return name_;
  }

private:
  std::string name_;
  boost::recursive_mutex dr_server_mutex_;
  dynamic_reconfigure::Server<ConfigType> dr_server_;
  ConfigType config_;
};

template <typename ZividSettingsType, typename SettingsConfigType, typename SettingsAcquisitionConfigType>
CaptureSettingsController<ZividSettingsType, SettingsConfigType,
                          SettingsAcquisitionConfigType>::CaptureSettingsController(ros::NodeHandle& nh,
                                                                                    Zivid::Camera& camera,
                                                                                    const std::string& node_name,
                                                                                    std::size_t num_acquisition_servers)
  : node_name_(node_name)
{
  general_config_dr_server_ = std::make_unique<SettingsConfigTypeDRServer>(
      node_name_, nh, Zivid::Experimental::SettingsInfo::defaultValue<ZividSettingsType>(camera.info()));

  ROS_INFO("Setting up %ld %s/acquisition_<n> dynamic_reconfigure servers", num_acquisition_servers, node_name.c_str());
  for (std::size_t i = 0; i < num_acquisition_servers; i++)
  {
    acquisition_config_dr_servers_.push_back(std::make_unique<SettingsAcquisitionConfigTypeDRServer>(
        node_name + "/acquisition_" + std::to_string(i), nh,
        Zivid::Experimental::SettingsInfo::defaultValue<typename ZividSettingsType::Acquisition>(camera.info())));
  }
}

template <typename ZividSettingsType, typename SettingsConfigType, typename SettingsAcquisitionConfigType>
CaptureSettingsController<ZividSettingsType, SettingsConfigType,
                          SettingsAcquisitionConfigType>::~CaptureSettingsController() = default;

template <typename ZividSettingsType, typename SettingsConfigType, typename SettingsAcquisitionConfigType>
ZividSettingsType
CaptureSettingsController<ZividSettingsType, SettingsConfigType, SettingsAcquisitionConfigType>::zividSettings() const
{
  ZividSettingsType settings;
  applyConfigToZividSettings(general_config_dr_server_->config(), settings);

  for (const auto& dr_config_server : acquisition_config_dr_servers_)
  {
    if (dr_config_server->config().enabled)
    {
      ROS_DEBUG("Config %s is enabled", dr_config_server->name().c_str());
      typename ZividSettingsType::Acquisition acquisition;
      applyConfigToZividSettings(dr_config_server->config(), acquisition);
      settings.acquisitions().emplaceBack(std::move(acquisition));
    }
  }
  return settings;
}

template <typename ZividSettingsType, typename SettingsConfigType, typename SettingsAcquisitionConfigType>
void CaptureSettingsController<ZividSettingsType, SettingsConfigType, SettingsAcquisitionConfigType>::setZividSettings(
    const ZividSettingsType& settings)
{
  const auto& acquisitions = settings.acquisitions();

  if (acquisitions.size() > numAcquisitionConfigServers())
  {
    std::stringstream error;
    error << "The number of acquisitions (" + std::to_string(acquisitions.size()) + ") "
          << "is larger than the number of dynamic_reconfigure " + node_name_ + "/acquisition_<n> servers ("
          << std::to_string(numAcquisitionConfigServers()) << "). ";
    if constexpr (std::is_same_v<ZividSettingsType, Zivid::Settings>)
    {
      error << "Increase launch parameter max_capture_acquisitions. "
            << "See README.md for more information.";
    }
    else if constexpr (std::is_same_v<ZividSettingsType, Zivid::Settings2D>)
    {
      error << "In 2D mode only a single acquisiton is supported.";
    }
    throw std::runtime_error(error.str());
  }

  ROS_DEBUG_STREAM("Updating settings to " << settings);
  general_config_dr_server_->setConfig(zividSettingsToConfig<SettingsConfigType>(settings));

  for (std::size_t i = 0; i < acquisitions.size(); i++)
  {
    auto config = zividSettingsToConfig<SettingsAcquisitionConfigType>(acquisitions.at(i));
    config.enabled = true;
    acquisition_config_dr_servers_[i]->setConfig(config);
  }

  // Any other acquisition that are enabled must be disabled
  for (std::size_t i = acquisitions.size(); i < acquisition_config_dr_servers_.size(); i++)
  {
    if (acquisition_config_dr_servers_[i]->config().enabled)
    {
      ROS_INFO_STREAM("Acquisition " << i << " was enabled, so disabling it");
      auto config = acquisition_config_dr_servers_[i]->config();
      config.enabled = false;
      acquisition_config_dr_servers_[i]->setConfig(config);
    }
  }
}

template class CaptureSettingsController<Zivid::Settings, zivid_camera::SettingsConfig,
                                         zivid_camera::SettingsAcquisitionConfig>;
template class CaptureSettingsController<Zivid::Settings2D, zivid_camera::Settings2DConfig,
                                         zivid_camera::Settings2DAcquisitionConfig>;

}  // namespace zivid_camera
