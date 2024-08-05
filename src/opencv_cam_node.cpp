#include "opencv_cam/opencv_cam_node.hpp"

#include <iostream>

#include "camera_calibration_parsers/parse.hpp"

namespace opencv_cam
{

  std::string mat_type2encoding(int mat_type)
  {
    switch (mat_type)
    {
    case CV_8UC1:
      return "mono8";
    case CV_8UC3:
      return "bgr8";
    case CV_16SC1:
      return "mono16";
    case CV_8UC4:
      return "rgba8";
    default:
      throw std::runtime_error("unsupported encoding type");
    }
  }

  OpencvCamNode::OpencvCamNode(const rclcpp::NodeOptions &options)
      : Node("opencv_cam", options), canceled_(false)
  {
    RCLCPP_INFO(get_logger(), "use_intra_process_comms=%d",
                options.use_intra_process_comms());

    // Initialize parameters
#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) \
  CXT_MACRO_LOAD_PARAMETER((*this), cxt_, n, t, d)
    CXT_MACRO_INIT_PARAMETERS(OPENCV_CAM_ALL_PARAMS, validate_parameters)

    // Register for parameter changed. NOTE at this point nothing is done when
    // parameters change.
#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_PARAMETER_CHANGED(n, t)
    CXT_MACRO_REGISTER_PARAMETERS_CHANGED((*this), cxt_, OPENCV_CAM_ALL_PARAMS,
                                          validate_parameters)

    // Log the current parameters
#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_LOG_SORTED_PARAMETER(cxt_, n, t, d)
    CXT_MACRO_LOG_SORTED_PARAMETERS(
        RCLCPP_INFO, get_logger(), "opencv_cam Parameters", OPENCV_CAM_ALL_PARAMS)

    // Check that all command line parameters are registered
#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_CHECK_CMDLINE_PARAMETER(n, t, d)
    CXT_MACRO_CHECK_CMDLINE_PARAMETERS((*this), OPENCV_CAM_ALL_PARAMS)

    RCLCPP_INFO(get_logger(), "OpenCV version %d", CV_VERSION_MAJOR);

    // Open file or device
    if (cxt_.file_)
    {
      capture_ = std::make_shared<cv::VideoCapture>(cxt_.filename_);

      if (!capture_->isOpened())
      {
        RCLCPP_ERROR(get_logger(), "cannot open file %s", cxt_.filename_.c_str());
        return;
      }

      if (cxt_.fps_ > 0)
      {
        // Publish at the specified rate
        publish_fps_ = cxt_.fps_;
      }
      else
      {
        // Publish at the recorded rate
        publish_fps_ = static_cast<int>(capture_->get(cv::CAP_PROP_FPS));
      }

      double width = capture_->get(cv::CAP_PROP_FRAME_WIDTH);
      double height = capture_->get(cv::CAP_PROP_FRAME_HEIGHT);
      RCLCPP_INFO(get_logger(),
                  "file %s open, width %g, height %g, publish fps %d",
                  cxt_.filename_.c_str(), width, height, publish_fps_);

      next_stamp_ = now();
    }
    else
    {
      capture_ = std::make_shared<cv::VideoCapture>(cxt_.index_);

      if (!capture_->isOpened())
      {
        RCLCPP_ERROR(get_logger(), "cannot open device %d", cxt_.index_);
        return;
      }

      if (cxt_.height_ > 0)
      {
        capture_->set(cv::CAP_PROP_FRAME_HEIGHT, cxt_.height_);
      }

      if (cxt_.width_ > 0)
      {
        capture_->set(cv::CAP_PROP_FRAME_WIDTH, cxt_.width_);
      }

      if (cxt_.fps_ > 0)
      {
        capture_->set(cv::CAP_PROP_FPS, cxt_.fps_);
      }

      double width = capture_->get(cv::CAP_PROP_FRAME_WIDTH);

      half_width = width / 2;
      double height = capture_->get(cv::CAP_PROP_FRAME_HEIGHT);
      double fps = capture_->get(cv::CAP_PROP_FPS);
      RCLCPP_INFO(get_logger(),
                  "device %d open, width %g, height %g, device fps %g",
                  cxt_.index_, width, height, fps);
    }

    assert(!cxt_.camera_info_path_
                .empty()); // readCalibration will crash if file_name is ""
    std::string camera_name;
    if (camera_calibration_parsers::readCalibration(
            cxt_.camera_info_path_, camera_name, camera_info_msg_))
    {
      RCLCPP_INFO(get_logger(), "got camera info for '%s'", camera_name.c_str());
      camera_info_msg_.header.frame_id = cxt_.camera_frame_id_;
      camera_info_pub_ =
          create_publisher<sensor_msgs::msg::CameraInfo>("camera_info", 10);
    }
    else
    {
      RCLCPP_ERROR(get_logger(), "cannot get camera info, will not publish");
      camera_info_pub_ = nullptr;
    }
    image_pub_ = image_transport::create_publisher(this, "image_raw");
    if (cxt_.split_frame_)
    {
      image_pub2_ = image_transport::create_publisher(this, "image_raw2");
    }

    // Run loop on it's own thread
    thread_ = std::thread(std::bind(&OpencvCamNode::loop, this));

    RCLCPP_INFO(get_logger(), "start publishing");
  }

  OpencvCamNode::~OpencvCamNode()
  {
    // Stop loop
    canceled_.store(true);
    if (thread_.joinable())
    {
      thread_.join();
    }
  }

  void OpencvCamNode::validate_parameters() {}

  void OpencvCamNode::loop()
  {

    cv::Mat frame;
    while (rclcpp::ok() && !canceled_.load())
    {
      // Read a frame, if this is a device block until a frame is available
      if (!capture_->read(frame))
      {
        RCLCPP_INFO(get_logger(), "EOF, stop publishing");
        break;
      }

      auto stamp = now();
      
      if (cxt_.flip_)
      {
        cv::Mat dst;
        cv::flip(frame, dst, -1);
        dst.copyTo(frame);
      }

      if (cxt_.split_frame_)
      {
        cv::Rect roi_left(0, 0, half_width, frame.rows);
        cv::Rect roi_right(half_width, 0, half_width, frame.rows);

        cv::Mat left(frame, roi_left);
        cv::Mat right(frame.rows, half_width, frame.type());
        frame(roi_right).copyTo(right);

        create_and_publish_image(stamp, left, cxt_.camera_frame_id_, image_pub_);
        create_and_publish_image(stamp, right, cxt_.camera_frame_id2_, image_pub2_);
      }
      else
      {

        create_and_publish_image(stamp, frame, cxt_.camera_frame_id_, image_pub_);
      }

#undef SHOW_ADDRESS
#ifdef SHOW_ADDRESS
      static int count = 0;
      RCLCPP_INFO(get_logger(), "%d, %p", count++,
                  reinterpret_cast<std::uintptr_t>(image_msg.get()));
#endif

      // Publish
      if (camera_info_pub_)
      {
        camera_info_msg_.header.stamp = stamp;
        camera_info_pub_->publish(camera_info_msg_);
      }

      // Sleep if required
      if (cxt_.file_)
      {
        using namespace std::chrono_literals;
        next_stamp_ = next_stamp_ + rclcpp::Duration{1000000000ns / publish_fps_};
        auto wait = next_stamp_ - stamp;
        if (wait.nanoseconds() > 0)
        {
          std::this_thread::sleep_for(
              static_cast<std::chrono::nanoseconds>(wait.nanoseconds()));
        }
      }
    }
  }

  void OpencvCamNode::create_and_publish_image(rclcpp::Time &stamp, cv::Mat &frame, std::string frame_id, image_transport::Publisher pub)
  {
    auto img = create_image_msg(stamp, frame, frame_id);
    pub.publish(std::move(img));
  }

  sensor_msgs::msg::Image::UniquePtr
  OpencvCamNode::create_image_msg(rclcpp::Time &stamp, cv::Mat &image,
                                  std::string frame_id)
  {
    sensor_msgs::msg::Image::UniquePtr ros_image(new sensor_msgs::msg::Image());

    ros_image->header.stamp = stamp;
    ros_image->header.frame_id = frame_id;
    ros_image->height = image.rows;
    ros_image->width = image.cols;
    ros_image->encoding = "bgr8";
    // ros_image->is_bigendian = (rcpputils::endian::native == rcpputils::endian::big);
    ros_image->step = image.cols * image.elemSize();
    size_t size = ros_image->step * image.rows;
    ros_image->data.resize(size);

    if (image.isContinuous())
    {
      memcpy(reinterpret_cast<char *>(&ros_image->data[0]), image.data, size);
    }
    else
    {
      // Copy by row by row
      uchar *ros_data_ptr = reinterpret_cast<uchar *>(&ros_image->data[0]);
      uchar *cv_data_ptr = image.data;
      for (int i = 0; i < image.rows; ++i)
      {
        memcpy(ros_data_ptr, cv_data_ptr, ros_image->step);
        ros_data_ptr += ros_image->step;
        cv_data_ptr += image.step;
      }
    }
    return ros_image;
  }
  sensor_msgs::msg::CompressedImage::UniquePtr
  OpencvCamNode::create_cimage_msg(rclcpp::Time &stamp, cv::Mat &frame,
                                   std::string frame_id, std::string compression)
  {
    sensor_msgs::msg::CompressedImage::UniquePtr image_msg(new sensor_msgs::msg::CompressedImage());

    image_msg->header.stamp = stamp;
    image_msg->header.frame_id = frame_id;
    image_msg->format = compression;
    cv::imencode("." + compression, frame, image_msg->data);

    return image_msg;
  }

} // namespace opencv_cam
#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(opencv_cam::OpencvCamNode)
