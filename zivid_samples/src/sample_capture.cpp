#include <zivid_camera/SettingsAcquisitionConfig.h>
#include <zivid_camera/SettingsConfig.h>
#include <zivid_camera/Capture.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/client.h>
#include <sensor_msgs/PointCloud2.h>
#include <ros/ros.h>

#define CHECK(cmd)                                                                                                     \
  do                                                                                                                   \
  {                                                                                                                    \
    if (!cmd)                                                                                                          \
    {                                                                                                                  \
      throw std::runtime_error{ "\"" #cmd "\" failed!" };                                                              \
    }                                                                                                                  \
  } while (false)

namespace
{
const ros::Duration default_wait_duration{ 30 };

auto capture_start_time = std::chrono::high_resolution_clock::now();

void capture()
{
  ROS_INFO("Calling capture service");
  zivid_camera::Capture capture;
  capture_start_time = std::chrono::high_resolution_clock::now();
  CHECK(ros::service::call("/zivid_camera/capture", capture));
}

void on_points(const sensor_msgs::PointCloud2ConstPtr&)
{
  ROS_INFO("PointCloud received");

  auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() -
                                                                        capture_start_time)
                      .count();
  ROS_INFO("Capture took %lu milliseconds", duration);

  capture();
}

}  // namespace

int main(int argc, char** argv)
{
  ros::init(argc, argv, "sample_capture_cpp");
  ros::NodeHandle n;

  ROS_INFO("Starting sample_capture.cpp");

  CHECK(ros::service::waitForService("/zivid_camera/capture", default_wait_duration));

  ros::AsyncSpinner spinner(1);
  spinner.start();

  auto points_sub = n.subscribe("/zivid_camera/points/xyzrgba", 1, on_points);

  dynamic_reconfigure::Client<zivid_camera::SettingsConfig> capture_general_client("/zivid_camera/"
                                                                                   "settings/");
  zivid_camera::SettingsConfig config;

  ROS_INFO("config.processing_filters_noise_removal_threshold=%.5f", config.processing_filters_noise_removal_threshold);

  CHECK(capture_general_client.getDefaultConfiguration(config, default_wait_duration));
  config.processing_filters_reflection_removal_enabled = false;
  CHECK(capture_general_client.setConfiguration(config));

  ROS_INFO("Enabling and configuring the first acquisition");
  dynamic_reconfigure::Client<zivid_camera::SettingsAcquisitionConfig> acquisition_0_client("/zivid_camera/settings/"
                                                                                            "acquisition_0/");

  zivid_camera::SettingsAcquisitionConfig acquisition_0_cfg;
  CHECK(acquisition_0_client.getDefaultConfiguration(acquisition_0_cfg, default_wait_duration));
  acquisition_0_cfg.enabled = true;
  acquisition_0_cfg.exposure_time = 6500;

  ROS_INFO("acquisition_0_cfg.enabled=%d", acquisition_0_cfg.enabled);
  ROS_INFO("acquisition_0_cfg.aperture=%.6f", acquisition_0_cfg.aperture);
  ROS_INFO("acquisition_0_cfg.exposure_time=%ul", acquisition_0_cfg.exposure_time);
  ROS_INFO("acquisition_0_cfg.gain=%.6f", acquisition_0_cfg.gain);
  ROS_INFO("acquisition_0_cfg.brightness=%.6f", acquisition_0_cfg.brightness);
  ROS_INFO("Calling acquisition_0_client.setConfiguration(acquisition_0_cfg)");

  CHECK(acquisition_0_client.setConfiguration(acquisition_0_cfg));

  ROS_INFO("Calling capture");

  capture();

  ros::waitForShutdown();

  return 0;
}