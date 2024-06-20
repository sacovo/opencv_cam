#ifndef OPENCV_CAM_HPP
#define OPENCV_CAM_HPP

#include "opencv2/highgui/highgui.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"

#include "image_transport/image_transport.hpp"

#include "opencv_cam/camera_context.hpp"

namespace opencv_cam
{

  class OpencvCamNode : public rclcpp::Node
  {
    CameraContext cxt_;

    std::thread thread_;
    std::atomic<bool> canceled_;

    std::shared_ptr<cv::VideoCapture> capture_;
    sensor_msgs::msg::CameraInfo camera_info_msg_;

    int publish_fps_;
    rclcpp::Time next_stamp_;

    double half_width;

    image_transport::Publisher image_pub_, image_pub2_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;

  public:
    explicit OpencvCamNode(const rclcpp::NodeOptions &options);

    ~OpencvCamNode() override;

  private:
    void validate_parameters();

    void loop();

    void create_and_publish_image(rclcpp::Time &stamp, cv::Mat &frame, std::string frame_id, image_transport::Publisher pub);

    sensor_msgs::msg::Image::UniquePtr create_image_msg(rclcpp::Time &stamp, cv::Mat &frame, std::string frame_id);
    sensor_msgs::msg::CompressedImage::UniquePtr create_cimage_msg(rclcpp::Time &stamp, cv::Mat &frame, std::string frame_id, std::string compression);
  };

} // namespace opencv_cam

#endif // OPENCV_CAM_HPP
