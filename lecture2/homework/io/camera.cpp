#include "camera.hpp"

#include <stdexcept>
#include <string>
#include <unordered_map>

Camera::Camera()
: handle_(nullptr)
{
  // 1. 枚举 USB 相机
  MV_CC_DEVICE_INFO_LIST device_list{};
  int ret = MV_CC_EnumDevices(MV_USB_DEVICE, &device_list);

  if (ret != MV_OK) {
    throw std::runtime_error(
      "MV_CC_EnumDevices failed, error code: " +
      std::to_string(ret));
  }

  if (device_list.nDeviceNum == 0) {
    throw std::runtime_error("No HikRobot camera found");
  }

  // 2. 为第一台相机创建句柄
  ret = MV_CC_CreateHandle(
    &handle_,
    device_list.pDeviceInfo[0]);

  if (ret != MV_OK) {
    handle_ = nullptr;

    throw std::runtime_error(
      "MV_CC_CreateHandle failed, error code: " +
      std::to_string(ret));
  }

  // 3. 打开相机
  ret = MV_CC_OpenDevice(handle_);

  if (ret != MV_OK) {
    MV_CC_DestroyHandle(handle_);
    handle_ = nullptr;

    throw std::runtime_error(
      "MV_CC_OpenDevice failed, error code: " +
      std::to_string(ret));
  }

  // 4. 设置相机参数
  MV_CC_SetEnumValue(
    handle_,
    "BalanceWhiteAuto",
    MV_BALANCEWHITE_AUTO_CONTINUOUS);

  MV_CC_SetEnumValue(
    handle_,
    "ExposureAuto",
    MV_EXPOSURE_AUTO_MODE_OFF);

  MV_CC_SetEnumValue(
    handle_,
    "GainAuto",
    MV_GAIN_MODE_OFF);

  // 曝光时间单位为微秒：10000 us = 10 ms
  MV_CC_SetFloatValue(handle_, "ExposureTime", 10000);

  MV_CC_SetFloatValue(handle_, "Gain", 20);
  MV_CC_SetFrameRate(handle_, 60);

  // 5. 开始取流
  ret = MV_CC_StartGrabbing(handle_);

  if (ret != MV_OK) {
    MV_CC_CloseDevice(handle_);
    MV_CC_DestroyHandle(handle_);
    handle_ = nullptr;

    throw std::runtime_error(
      "MV_CC_StartGrabbing failed, error code: " +
      std::to_string(ret));
  }
}

cv::Mat Camera::transfer(const MV_FRAME_OUT & raw)
{
  // 用 cv::Mat 临时包装 SDK 提供的原始缓冲区。
  cv::Mat raw_img(
    raw.stFrameInfo.nHeight,
    raw.stFrameInfo.nWidth,
    CV_8UC1,
    raw.pBufAddr);

  // 不同 Bayer 排列对应不同的颜色转换方式。
  const static std::unordered_map<
    MvGvspPixelType,
    cv::ColorConversionCodes> type_map = {
      {
        PixelType_Gvsp_BayerGR8,
        cv::COLOR_BayerGR2RGB,
      },
      {
        PixelType_Gvsp_BayerRG8,
        cv::COLOR_BayerRG2RGB,
      },
      {
        PixelType_Gvsp_BayerGB8,
        cv::COLOR_BayerGB2RGB,
      },
      {
        PixelType_Gvsp_BayerBG8,
        cv::COLOR_BayerBG2RGB,
      },
    };

  const auto pixel_type = raw.stFrameInfo.enPixelType;
  const auto conversion = type_map.find(pixel_type);

  if (conversion == type_map.end()) {
    throw std::runtime_error(
      "Unsupported camera pixel type: " +
      std::to_string(
        static_cast<unsigned int>(pixel_type)));
  }

  // color_img 拥有独立的图像数据。
  cv::Mat color_img;
  cv::cvtColor(
    raw_img,
    color_img,
    conversion->second);

  return color_img;
}

cv::Mat Camera::read()
{
  if (handle_ == nullptr) {
    throw std::runtime_error("Camera is not initialized");
  }

  MV_FRAME_OUT raw{};

  // 最多等待 100 ms。
  constexpr unsigned int timeout_ms = 100;

  int ret = MV_CC_GetImageBuffer(
    handle_,
    &raw,
    timeout_ms);

  if (ret != MV_OK) {
    throw std::runtime_error(
      "MV_CC_GetImageBuffer failed, error code: " +
      std::to_string(ret));
  }

  cv::Mat img;

  // 如果图像转换抛出异常，也必须归还 SDK 缓冲区。
  try {
    img = transfer(raw);
  } catch (...) {
    MV_CC_FreeImageBuffer(handle_, &raw);
    throw;
  }

  ret = MV_CC_FreeImageBuffer(handle_, &raw);

  if (ret != MV_OK) {
    throw std::runtime_error(
      "MV_CC_FreeImageBuffer failed, error code: " +
      std::to_string(ret));
  }

  return img;
}

Camera::~Camera()
{
  // 析构函数不应抛出异常，因此这里只负责尽力释放资源。
  if (handle_ == nullptr) {
    return;
  }

  MV_CC_StopGrabbing(handle_);
  MV_CC_CloseDevice(handle_);
  MV_CC_DestroyHandle(handle_);

  handle_ = nullptr;
}
