# Zivid ROS driver changelog

This project adheres to [Semantic Versioning](https://semver.org).

## 2.0.0 (WORK IN PROGRESS)
* Support for version 2.0 of the Zivid SDK. For a full list of changes in the SDK, see the
  [SDK Changelog](https://www.zivid.com/software/releases/2.0.0-beta-1+6b13d5ad-356/Changelog.md).
* The "points" topic has been renamed to "points/xyzrgba". The "contrast" value is no longer
  available in this message. Use the SNR image at topic "snr/image" instead.
* Added new topic "points/xyz". This is similar to "points/xyzrgba" except the color values
  are not included in the message. The message is compatible with pcl::PointXYZ.
* Added new "snr/image" topic. This topic contains the SNR values for all the pixels. SNR
  replaces the old "contrast" values from SDK 1.0.
* Changes to how capture settings are configured:
  * The available settings, and the names and paths of the settings, has been updated to match
    the SDK 2.0 API. Note that some settings have been removed in 2.0. In addition the default
    values for some settings have changed. See the [README](./README.md) for a table of the
    supported settings in 2.0. See the
    [SDK Changelog](https://www.zivid.com/software/releases/2.0.0-beta-1+6b13d5ad-356/Changelog.md)
    for all changes to settings.
  * The dynamic_reconfigure node "capture/general/" has been renamed to "settings/".
  * The dynamic_reconfigure nodes "capture/frame_<n>/" has been renamed to "settings/acquisition_<n>/".
  * The dynamic_reconfigure nodes "capture_2d/frame_0/" has been renamed to "settings_2d/acquisition_0/".
  * Added new dynamic_reconfigure node "settings_2d/". As of 2.0 this contains color balance settings
    for 2D.
  * TODO elaborate on Dynamic Reconfigure.
* The depth image topic has been renamed from "depth/image_raw" to "depth/image". This is to
  follow the recommendation in [REP-118: Depth Images](https://www.ros.org/reps/rep-0118.html).
* The launch parameter "max_capture_acquisitions" has been renamed to "num_capture_frames", to be
  consistent with the naming in the Zivid SDK.

## 1.0.0

* Added support for Capture Assistant. Added new service "capture_assistant/suggest_settings"
  that can be used to find suggested settings for your scene. Added sample_capture_assistant
  for C++ and Python. For more information, see README.md.
* Added support for 2D capture. Added new service "capture_2d" and new dynamic_reconfigure
  server "capture_2d/frame_0". Added sample_capture_2d for both C++ and Python. For more
  information, see README.md.
* Made common sample.launch script for all samples.
* Adjusted rviz/camera_view.rviz to make the 2D RGB image a bit bigger.
* Other minor improvements to README.md.
* Bumped minimum Zivid SDK version to 1.7.0.

## 0.9.0

* Initial release of ROS driver for Zivid cameras. For more information, see README.md.
