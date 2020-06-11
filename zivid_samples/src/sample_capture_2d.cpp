#include <zivid_camera/Settings2DAcquisitionConfig.h>
#include <zivid_camera/Capture2D.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/client.h>
#include <sensor_msgs/Image.h>
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

void capture()
{
  ROS_INFO("Calling capture_2d service");
  zivid_camera::Capture2D capture_2d;
  CHECK(ros::service::call("/zivid_camera/capture_2d", capture_2d));
}

void on_image_color(const sensor_msgs::ImageConstPtr&)
{
  ROS_INFO("2D color image received");
  capture();
}

}  // namespace

int main(int argc, char** argv)
{
  ros::init(argc, argv, "sample_capture_2d");
  ros::NodeHandle n;

  ROS_INFO("Starting sample_capture_2d.cpp");

  CHECK(ros::service::waitForService("/zivid_camera/capture_2d", default_wait_duration));

  ros::AsyncSpinner spinner(1);
  spinner.start();

  auto image_color_sub = n.subscribe("/zivid_camera/color/image_color", 1, on_image_color);

  ROS_INFO("Configuring image settings");
  dynamic_reconfigure::Client<zivid_camera::Settings2DAcquisitionConfig> acquisition_0_client("/zivid_camera/"
                                                                                              "settings_2d/"
                                                                                              "acquisition_0/");

  zivid_camera::Settings2DAcquisitionConfig acquisition_0_cfg;
  CHECK(acquisition_0_client.getDefaultConfiguration(acquisition_0_cfg, default_wait_duration));

  acquisition_0_cfg.enabled = true;
  acquisition_0_cfg.aperture = 5.66;
  acquisition_0_cfg.exposure_time = 10000;
  acquisition_0_cfg.brightness = 1.0;
  acquisition_0_cfg.gain = 1.0;
  CHECK(acquisition_0_client.setConfiguration(acquisition_0_cfg));

  capture();

  ros::waitForShutdown();

  return 0;
}