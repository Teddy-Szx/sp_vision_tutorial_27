#ifndef IO__CAMERA_HPP
#define IO__CAMERA_HPP

#include <opencv2/opencv.hpp>

#include "hikrobot/include/MvCameraControl.h"

class Camera
{
public:
  Camera();
  ~Camera();
  cv::Mat read();

private:
  void * handle_;

  cv::Mat transfer(const MV_FRAME_OUT & raw);
};

#endif  // IO__CAMERA_HPP
