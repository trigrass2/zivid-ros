#ifdef __clang__
#pragma clang diagnostic push
// Errors to ignore for this entire file
#pragma clang diagnostic ignored "-Wglobal-constructors"  // error triggered by gtest fixtures
#endif

#include <zivid_camera/CameraInfoSerialNumber.h>
#include <zivid_camera/CameraInfoModelName.h>
#include <zivid_camera/Capture.h>
#include <zivid_camera/Capture2D.h>
#include <zivid_camera/CaptureAssistantSuggestSettings.h>
#include <zivid_camera/SettingsAcquisitionConfig.h>
#include <zivid_camera/Settings2DAcquisitionConfig.h>
#include <zivid_camera/SettingsConfig.h>
#include <zivid_camera/IsConnected.h>

#include <Zivid/Application.h>
#include <Zivid/CaptureAssistant.h>
#include <Zivid/Frame.h>
#include <Zivid/Camera.h>
#include <Zivid/Version.h>

#include <dynamic_reconfigure/client.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>

#include "gtest_include_wrapper.h"

#include <ros/ros.h>

using SecondsD = std::chrono::duration<double>;

namespace
{
template <class Rep, class Period>
ros::Duration toRosDuration(const std::chrono::duration<Rep, Period>& d)
{
  return ros::Duration{ std::chrono::duration_cast<SecondsD>(d).count() };
}
}  // namespace

class ZividNodeTest : public testing::Test
{
protected:
  ros::NodeHandle nh_;

  const ros::Duration node_ready_wait_duration{ 15 };
  const ros::Duration short_wait_duration{ 0.25 };
  const ros::Duration dr_get_max_wait_duration{ 5 };
  static constexpr auto capture_service_name = "/zivid_camera/capture";
  static constexpr auto capture_2d_service_name = "/zivid_camera/capture_2d";
  static constexpr auto capture_assistant_suggest_settings_service_name = "/zivid_camera/capture_assistant/"
                                                                          "suggest_settings";
  static constexpr auto color_camera_info_topic_name = "/zivid_camera/color/camera_info";
  static constexpr auto color_image_color_topic_name = "/zivid_camera/color/image_color";
  static constexpr auto depth_camera_info_topic_name = "/zivid_camera/depth/camera_info";
  static constexpr auto depth_image_raw_topic_name = "/zivid_camera/depth/image";
  static constexpr auto snr_image_topic_name = "/zivid_camera/snr/image";
  static constexpr auto points_xyz_topic_name = "/zivid_camera/points/xyz";
  static constexpr auto points_xyzrgba_topic_name = "/zivid_camera/points/xyzrgba";
  static constexpr size_t num_dr_capture_servers = 10;
  static constexpr auto file_camera_path = "/usr/share/Zivid/data/FileCameraZividOne.zfc";

  class SubscriptionWrapper
  {
  public:
    template <class Type, class Fn>
    static SubscriptionWrapper make(ros::NodeHandle& nh, const std::string& name, Fn&& fn)
    {
      SubscriptionWrapper w;
      boost::function<void(const boost::shared_ptr<const Type>&)> cb = [ptr = w.num_messages_.get(),
                                                                        fn = std::move(fn)](const auto& v) mutable {
        (*ptr)++;
        fn(v);
      };
      w.subscriber_ = nh.subscribe<Type>(name, 1, cb);
      return w;
    }

    std::size_t numMessages() const
    {
      return *num_messages_;
    }

  private:
    SubscriptionWrapper() : num_messages_(std::make_unique<std::size_t>(0))
    {
    }
    ros::Subscriber subscriber_;
    std::unique_ptr<std::size_t> num_messages_;
  };

  void waitForReady()
  {
    ASSERT_TRUE(ros::service::waitForService(capture_service_name, node_ready_wait_duration));
  }

  void enableFirst3DAcquisition()
  {
    dynamic_reconfigure::Client<zivid_camera::SettingsAcquisitionConfig> acquisition_0_client("/zivid_camera/settings/"
                                                                                              "acquisition_0/");
    zivid_camera::SettingsAcquisitionConfig acquisition_0_cfg;
    ASSERT_TRUE(acquisition_0_client.getDefaultConfiguration(acquisition_0_cfg, dr_get_max_wait_duration));
    acquisition_0_cfg.enabled = true;
    ASSERT_TRUE(acquisition_0_client.setConfiguration(acquisition_0_cfg));
  }

  void enableFirst2DAcquisition()
  {
    dynamic_reconfigure::Client<zivid_camera::Settings2DAcquisitionConfig> acquisition_0_client("/zivid_camera/"
                                                                                                "settings_2d/"
                                                                                                "acquisition_0/");
    zivid_camera::Settings2DAcquisitionConfig cfg;
    ASSERT_TRUE(acquisition_0_client.getDefaultConfiguration(cfg, dr_get_max_wait_duration));
    cfg.enabled = true;
    ASSERT_TRUE(acquisition_0_client.setConfiguration(cfg));
  }

  void enableFirst3DAcquisitionAndCapture()
  {
    enableFirst3DAcquisition();
    zivid_camera::Capture capture;
    ASSERT_TRUE(ros::service::call(capture_service_name, capture));
    short_wait_duration.sleep();
  }

  template <class Type, class Fn>
  SubscriptionWrapper subscribe(const std::string& name, Fn&& callback)
  {
    return SubscriptionWrapper::make<Type>(nh_, name, callback);
  }

  template <class Type>
  SubscriptionWrapper subscribe(const std::string& name)
  {
    return subscribe<Type>(name, [](const auto&) {});
  }

  template <class A, class B>
  void assertArrayFloatEq(const A& actual, const B& expected)
  {
    ASSERT_EQ(actual.size(), expected.size());
    for (std::size_t i = 0; i < actual.size(); i++)
    {
      ASSERT_FLOAT_EQ(actual[i], expected[i]);
    }
  }

  void assertCameraInfoForFileCamera(const sensor_msgs::CameraInfo& ci)
  {
    ASSERT_EQ(ci.width, 1920U);
    ASSERT_EQ(ci.height, 1200U);
    ASSERT_EQ(ci.distortion_model, "plumb_bob");

    //     [fx  0 cx]
    // K = [ 0 fy cy]
    //     [ 0  0  1]
    assertArrayFloatEq(
        ci.K, std::array<double, 9>{ 2759.12329102, 0, 958.78460693, 0, 2758.73681641, 634.94018555, 0, 0, 1 });

    // R = I
    assertArrayFloatEq(ci.R, std::array<double, 9>{ 1, 0, 0, 0, 1, 0, 0, 0, 1 });

    //     [fx'  0  cx' Tx]
    // P = [ 0  fy' cy' Ty]
    //     [ 0   0   1   0]
    assertArrayFloatEq(ci.P, std::array<double, 12>{ 2759.12329102, 0, 958.78460693, 0, 0, 2758.73681641, 634.94018555,
                                                     0, 0, 0, 1, 0 });
  }
};

TEST_F(ZividNodeTest, testServiceCameraInfoModelName)
{
  waitForReady();
  zivid_camera::CameraInfoModelName model_name;
  ASSERT_TRUE(ros::service::call("/zivid_camera/camera_info/model_name", model_name));
  ASSERT_EQ(model_name.response.model_name, std::string("FileCamera-") + ZIVID_CORE_VERSION);
}

TEST_F(ZividNodeTest, testServiceCameraInfoSerialNumber)
{
  waitForReady();
  zivid_camera::CameraInfoSerialNumber serial_number;
  ASSERT_TRUE(ros::service::call("/zivid_camera/camera_info/serial_number", serial_number));
  ASSERT_EQ(serial_number.response.serial_number, "F1");
}

TEST_F(ZividNodeTest, testServiceIsConnected)
{
  waitForReady();
  zivid_camera::IsConnected is_connected;
  ASSERT_TRUE(ros::service::call("/zivid_camera/is_connected", is_connected));
  ASSERT_EQ(is_connected.response.is_connected, true);
}

TEST_F(ZividNodeTest, testCapturePublishesTopics)
{
  waitForReady();

  auto color_camera_info_sub = subscribe<sensor_msgs::CameraInfo>(color_camera_info_topic_name);
  auto color_image_color_sub = subscribe<sensor_msgs::Image>(color_image_color_topic_name);
  auto depth_camera_info_sub = subscribe<sensor_msgs::CameraInfo>(depth_camera_info_topic_name);
  auto depth_image_raw_sub = subscribe<sensor_msgs::Image>(depth_image_raw_topic_name);
  auto points_xyz_sub = subscribe<sensor_msgs::PointCloud2>(points_xyz_topic_name);
  auto points_xyzrgb_sub = subscribe<sensor_msgs::PointCloud2>(points_xyzrgba_topic_name);

  auto assert_num_topics_received = [&](std::size_t numTopics) {
    ASSERT_EQ(color_camera_info_sub.numMessages(), numTopics);
    ASSERT_EQ(color_image_color_sub.numMessages(), numTopics);
    ASSERT_EQ(depth_camera_info_sub.numMessages(), numTopics);
    ASSERT_EQ(depth_image_raw_sub.numMessages(), numTopics);
    ASSERT_EQ(points_xyz_sub.numMessages(), numTopics);
    ASSERT_EQ(points_xyzrgb_sub.numMessages(), numTopics);
  };

  short_wait_duration.sleep();
  assert_num_topics_received(0);

  zivid_camera::Capture capture;
  // Capture fails when no acquisitions are enabled
  ASSERT_FALSE(ros::service::call(capture_service_name, capture));
  short_wait_duration.sleep();
  assert_num_topics_received(0);

  enableFirst3DAcquisition();

  ASSERT_TRUE(ros::service::call(capture_service_name, capture));
  short_wait_duration.sleep();
  assert_num_topics_received(1);

  ASSERT_TRUE(ros::service::call(capture_service_name, capture));
  short_wait_duration.sleep();
  assert_num_topics_received(2);

  ASSERT_TRUE(ros::service::call(capture_service_name, capture));
  short_wait_duration.sleep();
  assert_num_topics_received(3);

  short_wait_duration.sleep();
  assert_num_topics_received(3);
}

class CaptureOutputTest : public ZividNodeTest
{
protected:
  CaptureOutputTest() : m_camera(m_zivid.createFileCamera(file_camera_path))
  {
    waitForReady();
  }

  Zivid::Frame captureViaSDKDefaultSettings()
  {
    return m_camera.capture(Zivid::Settings{ Zivid::Settings::Acquisitions{ Zivid::Settings::Acquisition{} } });
  }

  Zivid::Frame2D capture2DViaSDKDefaultSettings()
  {
    return m_camera.capture(Zivid::Settings2D{ Zivid::Settings2D::Acquisitions{ Zivid::Settings2D::Acquisition{} } });
  }

  void comparePointCoordinate(float rosCoordinate, float sdkCoordinate)
  {
    if (std::isnan(rosCoordinate))
    {
      ASSERT_TRUE(std::isnan(sdkCoordinate));
    }
    else
    {
      // Output from the SDK is millimeters, but in the ROS driver the points are transformed to meters.
      const float delta = 0.000001f;
      ASSERT_NEAR(rosCoordinate, sdkCoordinate / 1000, delta);
    }
  }

private:
  Zivid::Application m_zivid;
  Zivid::Camera m_camera;
};

TEST_F(CaptureOutputTest, testCapturePointsXYZGBA)
{
  std::optional<sensor_msgs::PointCloud2> last_pc2;
  auto points_sub =
      subscribe<sensor_msgs::PointCloud2>(points_xyzrgba_topic_name, [&](const auto& p) { last_pc2 = *p; });

  enableFirst3DAcquisitionAndCapture();

  ASSERT_TRUE(last_pc2.has_value());
  ASSERT_EQ(last_pc2->width, 1920U);
  ASSERT_EQ(last_pc2->height, 1200U);
  ASSERT_EQ(last_pc2->point_step, 16U);
  ASSERT_EQ(last_pc2->row_step, last_pc2->width * last_pc2->point_step);
  ASSERT_EQ(last_pc2->is_dense, false);
  ASSERT_EQ(last_pc2->data.size(), last_pc2->width * last_pc2->height * last_pc2->point_step);

  const auto frame = captureViaSDKDefaultSettings();
  const auto expectedXYZRGBA = frame.pointCloud().copyData<Zivid::PointXYZColorRGBA>();
  ASSERT_EQ(last_pc2->width, expectedXYZRGBA.width());
  ASSERT_EQ(last_pc2->height, expectedXYZRGBA.height());
  for (std::size_t i = 0; i < expectedXYZRGBA.size(); i++)
  {
    const auto index = i * last_pc2->point_step;
    uint8_t* point_ptr = &last_pc2->data[index];
    const float x = *reinterpret_cast<float*>(&(point_ptr[0]));
    const float y = *reinterpret_cast<float*>(&(point_ptr[4]));
    const float z = *reinterpret_cast<float*>(&(point_ptr[8]));
    const uint32_t argb = *reinterpret_cast<uint32_t*>(&(point_ptr[12]));
    const auto& expected = expectedXYZRGBA(i);

    comparePointCoordinate(x, expected.point.x);
    comparePointCoordinate(y, expected.point.y);
    comparePointCoordinate(z, expected.point.z);
    const auto expectedARGB =
        (expected.color.a << 24) | (expected.color.r << 16) | (expected.color.g << 8) | expected.color.b;
    ASSERT_EQ(argb, expectedARGB);
  }
}

TEST_F(CaptureOutputTest, testCapturePointsXYZ)
{
  std::optional<sensor_msgs::PointCloud2> point_cloud;
  auto points_sub =
      subscribe<sensor_msgs::PointCloud2>(points_xyz_topic_name, [&](const auto& p) { point_cloud = *p; });

  enableFirst3DAcquisitionAndCapture();

  ASSERT_TRUE(point_cloud.has_value());
  ASSERT_EQ(point_cloud->width, 1920U);
  ASSERT_EQ(point_cloud->height, 1200U);
  ASSERT_EQ(point_cloud->point_step, 16U);  // 3x4 bytes for xyz + 4 bytes padding (w)
  ASSERT_EQ(point_cloud->row_step, point_cloud->width * point_cloud->point_step);
  ASSERT_EQ(point_cloud->is_dense, false);
  ASSERT_EQ(point_cloud->data.size(), point_cloud->width * point_cloud->height * point_cloud->point_step);

  auto frame = captureViaSDKDefaultSettings();
  auto expectedXYZ = frame.pointCloud().copyData<Zivid::PointXYZ>();
  ASSERT_EQ(point_cloud->width, expectedXYZ.width());
  ASSERT_EQ(point_cloud->height, expectedXYZ.height());
  for (std::size_t i = 0; i < expectedXYZ.size(); i++)
  {
    const auto index = i * point_cloud->point_step;
    uint8_t* point_ptr = &point_cloud->data[index];
    const float x = *reinterpret_cast<float*>(&(point_ptr[0]));
    const float y = *reinterpret_cast<float*>(&(point_ptr[4]));
    const float z = *reinterpret_cast<float*>(&(point_ptr[8]));

    const auto expected = expectedXYZ(i);
    comparePointCoordinate(x, expected.x);
    comparePointCoordinate(y, expected.y);
    comparePointCoordinate(z, expected.z);
  }
}

TEST_F(CaptureOutputTest, testCapture3DColorImage)
{
  std::optional<sensor_msgs::Image> image;
  auto color_image_sub =
      subscribe<sensor_msgs::Image>(color_image_color_topic_name, [&](const auto& i) { image = *i; });

  enableFirst3DAcquisitionAndCapture();

  ASSERT_TRUE(image.has_value());
  ASSERT_EQ(image->width, 1920U);
  ASSERT_EQ(image->height, 1200U);
  constexpr uint32_t bytes_per_pixel = 4U;
  ASSERT_EQ(image->step, bytes_per_pixel * image->width);
  ASSERT_EQ(image->data.size(), image->step * image->height);
  ASSERT_EQ(image->encoding, "rgba8");
  ASSERT_EQ(image->is_bigendian, false);

  const auto frame = captureViaSDKDefaultSettings();
  const auto expectedRGBA = frame.pointCloud().copyData<Zivid::ColorRGBA>();
  ASSERT_EQ(image->width, expectedRGBA.width());
  ASSERT_EQ(image->height, expectedRGBA.height());
  for (std::size_t i = 0; i < expectedRGBA.size(); i++)
  {
    const auto expected = expectedRGBA(i);
    const auto index = i * bytes_per_pixel;
    ASSERT_EQ(image->data[index], expected.r);
    ASSERT_EQ(image->data[index + 1], expected.g);
    ASSERT_EQ(image->data[index + 2], expected.b);
    ASSERT_EQ(image->data[index + 3], expected.a);
    ASSERT_EQ(expected.a, 255);
  }
}

TEST_F(CaptureOutputTest, testCaptureDepthImage)
{
  std::optional<sensor_msgs::Image> image;
  auto depth_image_sub = subscribe<sensor_msgs::Image>(depth_image_raw_topic_name, [&](const auto& i) { image = *i; });

  enableFirst3DAcquisitionAndCapture();

  ASSERT_TRUE(image.has_value());
  ASSERT_EQ(image->width, 1920U);
  ASSERT_EQ(image->height, 1200U);
  constexpr uint32_t bytes_per_pixel = 4U;
  ASSERT_EQ(image->step, bytes_per_pixel * image->width);
  ASSERT_EQ(image->data.size(), image->step * image->height);
  ASSERT_EQ(image->encoding, "32FC1");
  ASSERT_EQ(image->is_bigendian, false);

  const auto frame = captureViaSDKDefaultSettings();
  const auto expectedZ = frame.pointCloud().copyData<Zivid::PointZ>();
  ASSERT_EQ(image->width, expectedZ.width());
  ASSERT_EQ(image->height, expectedZ.height());
  for (std::size_t i = 0; i < expectedZ.size(); i++)
  {
    const auto expected = expectedZ(i);
    const float z = *reinterpret_cast<float*>(image->data.data() + i * bytes_per_pixel);
    comparePointCoordinate(z, expected.z);
  }
}

TEST_F(CaptureOutputTest, testCaptureSNRImage)
{
  std::optional<sensor_msgs::Image> image;
  auto snr_image_sub = subscribe<sensor_msgs::Image>(snr_image_topic_name, [&](const auto& i) { image = *i; });

  enableFirst3DAcquisitionAndCapture();

  ASSERT_TRUE(image.has_value());
  ASSERT_EQ(image->width, 1920U);
  ASSERT_EQ(image->height, 1200U);
  constexpr uint32_t bytes_per_pixel = 4U;
  ASSERT_EQ(image->step, bytes_per_pixel * image->width);
  ASSERT_EQ(image->data.size(), image->step * image->height);
  ASSERT_EQ(image->encoding, "32FC1");
  ASSERT_EQ(image->is_bigendian, false);

  const auto frame = captureViaSDKDefaultSettings();
  const auto expectedSNR = frame.pointCloud().copyData<Zivid::SNR>();
  ASSERT_EQ(image->width, expectedSNR.width());
  ASSERT_EQ(image->height, expectedSNR.height());
  for (std::size_t i = 0; i < expectedSNR.size(); i++)
  {
    const auto expected = expectedSNR(i);
    const float snr = *reinterpret_cast<float*>(image->data.data() + i * bytes_per_pixel);
    ASSERT_EQ(snr, expected.value);
  }
}

TEST_F(ZividNodeTest, testCaptureCameraInfo)
{
  waitForReady();

  std::optional<sensor_msgs::CameraInfo> color_camera_info;
  auto color_camera_info_sub =
      subscribe<sensor_msgs::CameraInfo>(color_camera_info_topic_name, [&](const auto& r) { color_camera_info = *r; });

  std::optional<sensor_msgs::CameraInfo> depth_camera_info;
  auto depth_camera_info_sub =
      subscribe<sensor_msgs::CameraInfo>(depth_camera_info_topic_name, [&](const auto& r) { depth_camera_info = *r; });

  enableFirst3DAcquisitionAndCapture();

  ASSERT_EQ(color_camera_info_sub.numMessages(), 1U);
  ASSERT_EQ(depth_camera_info_sub.numMessages(), 1U);

  ASSERT_TRUE(color_camera_info.has_value());
  assertCameraInfoForFileCamera(*color_camera_info);
  ASSERT_TRUE(depth_camera_info.has_value());
  assertCameraInfoForFileCamera(*depth_camera_info);
}

TEST_F(ZividNodeTest, test3DSettingsDynamicReconfigureNodesAreAvailable)
{
  waitForReady();

  const std::string prefix = "/zivid_camera/settings/";

  ASSERT_TRUE(ros::service::waitForService(prefix + "set_parameters", short_wait_duration));
  for (std::size_t i = 0; i < 10U; i++)
  {
    ASSERT_TRUE(ros::service::waitForService(prefix + "acquisition_" + std::to_string(i) + "/set_parameters",
                                             short_wait_duration));
  }
  ASSERT_FALSE(ros::service::waitForService(prefix + "acquisition_11/set_parameters", short_wait_duration));
}

TEST_F(CaptureOutputTest, testCapture2D)
{
  std::optional<sensor_msgs::CameraInfo> color_camera_info;
  std::optional<sensor_msgs::Image> image;
  auto color_camera_info_sub =
      subscribe<sensor_msgs::CameraInfo>(color_camera_info_topic_name, [&](const auto& r) { color_camera_info = *r; });
  auto color_image_color_sub =
      subscribe<sensor_msgs::Image>(color_image_color_topic_name, [&](const auto& i) { image = *i; });

  auto assert_num_topics_received = [&](std::size_t numTopics) {
    ASSERT_EQ(color_camera_info_sub.numMessages(), numTopics);
    ASSERT_EQ(color_image_color_sub.numMessages(), numTopics);
  };

  short_wait_duration.sleep();
  assert_num_topics_received(0);

  // Capture fails when no acquisitions are enabled
  zivid_camera::Capture2D capture;
  ASSERT_FALSE(ros::service::call(capture_2d_service_name, capture));
  short_wait_duration.sleep();
  assert_num_topics_received(0);

  enableFirst2DAcquisition();
  ASSERT_TRUE(ros::service::call(capture_2d_service_name, capture));
  short_wait_duration.sleep();
  assert_num_topics_received(1);

  auto verifyImageAndCameraInfo = [this](const auto& img, const auto& info) {
    assertCameraInfoForFileCamera(info);

    ASSERT_EQ(img.width, 1920U);
    ASSERT_EQ(img.height, 1200U);
    constexpr uint32_t bytes_per_pixel = 4U;
    ASSERT_EQ(img.step, bytes_per_pixel * 1920U);
    ASSERT_EQ(img.encoding, "rgba8");
    ASSERT_EQ(img.is_bigendian, false);
    ASSERT_EQ(img.data.size(), img.step * img.height);

    const auto frame2D = capture2DViaSDKDefaultSettings();
    const auto expectedRGBA = frame2D.imageRGBA();
    ASSERT_EQ(img.width, expectedRGBA.width());
    ASSERT_EQ(img.height, expectedRGBA.height());
    for (std::size_t i = 0; i < expectedRGBA.size(); i++)
    {
      const auto expected = expectedRGBA(i);
      const auto index = i * bytes_per_pixel;
      ASSERT_EQ(img.data[index], expected.r);
      ASSERT_EQ(img.data[index + 1], expected.g);
      ASSERT_EQ(img.data[index + 2], expected.b);
      ASSERT_EQ(img.data[index + 3], expected.a);
      ASSERT_EQ(expected.a, 255);
    }
  };

  ASSERT_TRUE(image.has_value());
  ASSERT_TRUE(color_camera_info.has_value());
  verifyImageAndCameraInfo(*image, *color_camera_info);

  short_wait_duration.sleep();
  assert_num_topics_received(1);

  ASSERT_TRUE(ros::service::call(capture_2d_service_name, capture));
  short_wait_duration.sleep();
  assert_num_topics_received(2);
  verifyImageAndCameraInfo(*image, *color_camera_info);
}

TEST_F(ZividNodeTest, test2DSettingsDynamicReconfigureNodesAreAvailable)
{
  waitForReady();

  const std::string prefix = "/zivid_camera/settings_2d";
  ASSERT_TRUE(ros::service::waitForService(prefix + "/set_parameters", short_wait_duration));
  ASSERT_TRUE(ros::service::waitForService(prefix + "/acquisition_0/set_parameters", short_wait_duration));
  ASSERT_FALSE(ros::service::waitForService(prefix + "/acquisition_1/set_parameters", short_wait_duration));
}

class ZividCATest : public ZividNodeTest
{
protected:
  ZividCATest() : m_camera(m_zivid.createFileCamera(file_camera_path)), m_captureGeneralClient("/zivid_camera/settings")
  {
    waitForReady();
    m_settingsAcquisitionClients.reserve(num_dr_capture_servers);
    for (std::size_t i = 0; i < num_dr_capture_servers; i++)
    {
      using Client = dynamic_reconfigure::Client<zivid_camera::SettingsAcquisitionConfig>;
      m_settingsAcquisitionClients.emplace_back(
          std::make_unique<Client>("/zivid_camera/settings/acquisition_" + std::to_string(i)));
    }
  }

  zivid_camera::SettingsConfig settingsConfig()
  {
    zivid_camera::SettingsConfig cfg;
    EXPECT_TRUE(m_captureGeneralClient.getCurrentConfiguration(cfg, dr_get_max_wait_duration));
    return cfg;
  }

  zivid_camera::SettingsAcquisitionConfig settingsAcquisitionConfig(std::size_t i) const
  {
    zivid_camera::SettingsAcquisitionConfig cfg;
    EXPECT_TRUE(m_settingsAcquisitionClients[i]->getCurrentConfiguration(cfg, dr_get_max_wait_duration));
    return cfg;
  }

  std::size_t numEnabled3DAcquisitions() const
  {
    std::size_t enabled_acquisitions = 0;
    for (std::size_t i = 0; i < num_dr_capture_servers; i++)
    {
      if (settingsAcquisitionConfig(i).enabled)
      {
        enabled_acquisitions++;
      }
    }
    return enabled_acquisitions;
  }

  void compareSettingsAcquisitionConfigWithSettings(const Zivid::Settings::Acquisition& a,
                                                    const zivid_camera::SettingsAcquisitionConfig& cfg) const
  {
    ASSERT_EQ(true, cfg.enabled);
    ASSERT_EQ(a.aperture().value(), cfg.aperture);
    ASSERT_EQ(a.brightness().value(), cfg.brightness);
    ASSERT_EQ(a.exposureTime().value().count(), cfg.exposure_time);
    ASSERT_EQ(a.gain().value(), cfg.gain);
    ASSERT_EQ(a.patterns().sine().bidirectional().value(), cfg.patterns_sine_bidirectional);
  }

  void compareSettingsConfigWithSettings(const Zivid::Settings& s, const zivid_camera::SettingsConfig& cfg) const
  {
    const auto& color = s.processing().color();
    ASSERT_EQ(color.balance().blue().value(), cfg.processing_color_balance_blue);
    ASSERT_EQ(color.balance().green().value(), cfg.processing_color_balance_green);
    ASSERT_EQ(color.balance().red().value(), cfg.processing_color_balance_red);

    const auto& filters = s.processing().filters();
    ASSERT_EQ(filters.noise().removal().isEnabled().value(), cfg.processing_filters_noise_removal_enabled);
    ASSERT_EQ(filters.noise().removal().threshold().value(), cfg.processing_filters_noise_removal_threshold);
    ASSERT_EQ(filters.smoothing().gaussian().isEnabled().value(), cfg.processing_filters_smoothing_gaussian_enabled);
    ASSERT_EQ(filters.smoothing().gaussian().sigma().value(), cfg.processing_filters_smoothing_gaussian_sigma);
    ASSERT_EQ(filters.outlier().removal().isEnabled().value(), cfg.processing_filters_outlier_removal_enabled);
    ASSERT_EQ(filters.outlier().removal().threshold().value(), cfg.processing_filters_outlier_removal_threshold);
    ASSERT_EQ(filters.reflection().removal().isEnabled().value(), cfg.processing_filters_reflection_removal_enabled);
  }

  Zivid::CaptureAssistant::SuggestSettingsParameters::AmbientLightFrequency toAPIAmbientLightFrequency(
      zivid_camera::CaptureAssistantSuggestSettings::Request::_ambient_light_frequency_type ambient_light_frequency)
  {
    using AmbientLightFrequency = Zivid::CaptureAssistant::SuggestSettingsParameters::AmbientLightFrequency;
    using Request = zivid_camera::CaptureAssistantSuggestSettings::Request;
    switch (ambient_light_frequency)
    {
      case Request::AMBIENT_LIGHT_FREQUENCY_NONE:
        return AmbientLightFrequency::none;
      case Request::AMBIENT_LIGHT_FREQUENCY_50HZ:
        return AmbientLightFrequency::hz50;
      case Request::AMBIENT_LIGHT_FREQUENCY_60HZ:
        return AmbientLightFrequency::hz60;
    }
    throw std::runtime_error("Could not convert value " + std::to_string(ambient_light_frequency) + " to API enum.");
  }

  void performSuggestSettingsAndCompareWithCppAPI(
      ros::Duration max_capture_time,
      zivid_camera::CaptureAssistantSuggestSettings::Request::_ambient_light_frequency_type ambient_light_frequency)
  {
    zivid_camera::CaptureAssistantSuggestSettings srv;
    srv.request.max_capture_time = max_capture_time;
    srv.request.ambient_light_frequency = ambient_light_frequency;
    ASSERT_TRUE(ros::service::call(capture_assistant_suggest_settings_service_name, srv));
    short_wait_duration.sleep();

    Zivid::CaptureAssistant::SuggestSettingsParameters suggest_settings_parameters{
      Zivid::CaptureAssistant::SuggestSettingsParameters::MaxCaptureTime{
          std::chrono::round<std::chrono::milliseconds>(SecondsD{ max_capture_time.toSec() }) },
      toAPIAmbientLightFrequency(ambient_light_frequency)
    };
    const auto api_settings = Zivid::CaptureAssistant::suggestSettings(m_camera, suggest_settings_parameters);
    const auto& acquisitions = api_settings.acquisitions();

    ASSERT_EQ(acquisitions.size(), numEnabled3DAcquisitions());

    compareSettingsConfigWithSettings(api_settings, settingsConfig());
    for (std::size_t i = 0; i < acquisitions.size(); i++)
    {
      compareSettingsAcquisitionConfigWithSettings(acquisitions[i], settingsAcquisitionConfig(i));
    }
    for (std::size_t i = acquisitions.size(); i < num_dr_capture_servers; i++)
    {
      ASSERT_EQ(false, settingsAcquisitionConfig(i).enabled);
    }
  }

private:
  Zivid::Application m_zivid;
  Zivid::Camera m_camera;
  dynamic_reconfigure::Client<zivid_camera::SettingsConfig> m_captureGeneralClient;
  std::vector<std::unique_ptr<dynamic_reconfigure::Client<zivid_camera::SettingsAcquisitionConfig>>>
      m_settingsAcquisitionClients;
};

TEST_F(ZividCATest, testCaptureAssistantServiceAvailable)
{
  ASSERT_TRUE(ros::service::waitForService(capture_assistant_suggest_settings_service_name, short_wait_duration));
}

TEST_F(ZividCATest, testDifferentMaxCaptureTimeAndAmbientLightFrequency)
{
  using Request = zivid_camera::CaptureAssistantSuggestSettings::Request;
  for (double max_capture_time : { 0.2, 1.2, 10.0 })
  {
    for (auto ambient_light_frequency : { Request::AMBIENT_LIGHT_FREQUENCY_NONE, Request::AMBIENT_LIGHT_FREQUENCY_50HZ,
                                          Request::AMBIENT_LIGHT_FREQUENCY_60HZ })
    {
      performSuggestSettingsAndCompareWithCppAPI(ros::Duration{ max_capture_time }, ambient_light_frequency);
    }
  }
}

TEST_F(ZividCATest, testGoingFromMultipleAcquisitionsTo1Acquisition)
{
  using Request = zivid_camera::CaptureAssistantSuggestSettings::Request;
  performSuggestSettingsAndCompareWithCppAPI(ros::Duration{ 10.0 }, Request::AMBIENT_LIGHT_FREQUENCY_NONE);
  ASSERT_GT(numEnabled3DAcquisitions(), 1U);

  performSuggestSettingsAndCompareWithCppAPI(ros::Duration{ 0.2 }, Request::AMBIENT_LIGHT_FREQUENCY_NONE);
  ASSERT_EQ(numEnabled3DAcquisitions(), 1U);
}

TEST_F(ZividCATest, testCaptureAssistantWithInvalidMaxCaptureTimeFails)
{
  zivid_camera::CaptureAssistantSuggestSettings srv;
  srv.request.max_capture_time = ros::Duration{ 0.0 };
  ASSERT_FALSE(ros::service::call(capture_assistant_suggest_settings_service_name, srv));

  const auto validRange = Zivid::CaptureAssistant::SuggestSettingsParameters::MaxCaptureTime::validRange();
  const auto smallDelta = std::chrono::milliseconds{ 1 };
  srv.request.max_capture_time = toRosDuration(validRange.min() - smallDelta);
  ASSERT_FALSE(ros::service::call(capture_assistant_suggest_settings_service_name, srv));
  srv.request.max_capture_time = toRosDuration(validRange.max() + smallDelta);
  ASSERT_FALSE(ros::service::call(capture_assistant_suggest_settings_service_name, srv));

  srv.request.max_capture_time = toRosDuration(validRange.max());
  ASSERT_TRUE(ros::service::call(capture_assistant_suggest_settings_service_name, srv));
}

TEST_F(ZividCATest, testCaptureAssistantDefaultAmbientLightFrequencyWorks)
{
  zivid_camera::CaptureAssistantSuggestSettings srv;
  srv.request.max_capture_time = ros::Duration{ 1.0 };
  ASSERT_TRUE(ros::service::call(capture_assistant_suggest_settings_service_name, srv));
}

TEST_F(ZividCATest, testCaptureAssistantInvalidAmbientLightFrequencyFails)
{
  zivid_camera::CaptureAssistantSuggestSettings srv;
  srv.request.max_capture_time = ros::Duration{ 1.0 };
  srv.request.ambient_light_frequency = 255;
  ASSERT_FALSE(ros::service::call(capture_assistant_suggest_settings_service_name, srv));

  srv.request.ambient_light_frequency =
      zivid_camera::CaptureAssistantSuggestSettings::Request::AMBIENT_LIGHT_FREQUENCY_NONE;
  ASSERT_TRUE(ros::service::call(capture_assistant_suggest_settings_service_name, srv));
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_zivid_camera");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::NodeHandle nh;
  return RUN_ALL_TESTS();
}
