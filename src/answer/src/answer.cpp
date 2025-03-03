#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class HomeworkProcessor : public rclcpp::Node {
public:
  HomeworkProcessor() : Node("homework_processor") {
    // 创建订阅器（注意话题名称验证）
    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/homework/image_raw", 10,
      std::bind(&HomeworkProcessor::imageCallback, this, std::placeholders::_1));

    // 初始化OpenCV窗口（可选）
    cv::namedWindow("Binary Preview", cv::WINDOW_AUTOSIZE);
  }

  ~HomeworkProcessor() {
    cv::destroyAllWindows();
  }

private:
  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg) {
    try {
      // 转换ROS消息到OpenCV格式
      cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "mono8"); // 直接读取为灰度图

      // 二值化处理流程
      cv::Mat binary;
      cv::threshold(cv_ptr->image, binary, 128, 255, cv::THRESH_BINARY);

      // 打印核心信息
      printBinarySummary(binary);

      // 可视化调试输出
      cv::imshow("Binary Preview", binary);
      cv::waitKey(1);

    } catch (const cv_bridge::Exception& e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge异常: %s", e.what());
    }
  }

  void printBinarySummary(const cv::Mat& binary) {
    // 打印矩阵元数据
    RCLCPP_INFO(this->get_logger(),
      "收到图像 尺寸: %dx%d 通道数: %d 类型: %d",
      binary.cols, binary.rows, binary.channels(), binary.type());

    // 提取并打印示例数据块
    const int sample_size = 10;
    if(binary.rows > sample_size && binary.cols > sample_size) {
      std::ostringstream oss;
      oss << "中心区域样例:\n";
      cv::Rect center_roi(
        binary.cols/2 - sample_size/2,
        binary.rows/2 - sample_size/2,
        sample_size,
        sample_size
      );

      cv::Mat sample = binary(center_roi);
      for(int i = 0; i < sample.rows; ++i) {
        for(int j = 0; j < sample.cols; ++j) {
          oss << (sample.at<uchar>(i,j) > 0 ? "1" : "0") << " ";
        }
        oss << "\n";
      }
      RCLCPP_DEBUG(this->get_logger(), "%s", oss.str().c_str());
    }

    // 统计有效像素比例
    int white_pixels = cv::countNonZero(binary);
    RCLCPP_INFO(this->get_logger(),
      "有效像素占比: %.2f%%",
      (white_pixels * 100.0) / (binary.rows * binary.cols));
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<HomeworkProcessor>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
