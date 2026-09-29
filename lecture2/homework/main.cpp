#include "io/camera.hpp"
#include "tasks/yolo.hpp"
#include "tools/img_tools.hpp"

#include <exception>
#include <iostream>

#include <opencv2/opencv.hpp>

int main()
{
  try {
    Camera camera;
    auto_aim::YOLO yolo("./configs/yolo.yaml");

    int frame_count = 0;

    while (true) {
      // 1. 从相机读取一帧图像。
      cv::Mat img = camera.read();

      // 2. 使用 YOLO 识别图像中的装甲板。
      const auto armors = yolo.detect(img, frame_count++);

      // 3. 用绿色线条连接每块装甲板的四个关键点。
      for (const auto & armor : armors) {
        tools::draw_points(
          img,
          armor.points,
          cv::Scalar(0, 255, 0),
          2);
      }

      // 4. 缩放后显示结果。
      cv::resize(img, img, cv::Size(640, 480));
      cv::imshow("img", img);

      // waitKey(1) 保证画面连续更新，按 q 退出。
      const int key = cv::waitKey(1);
      if (key == 'q' || key == 'Q') {
        break;
      }
    }
  } catch (const std::exception & error) {
    std::cerr << "Program error: " << error.what() << '\n';
    return 1;
  }

  return 0;
}
