#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/packet_pipeline.h>
#include <libfreenect2/registration.h>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstring>
#include <memory>
#include <limits>
#include <stdexcept>
#include <string>
#include <thread>

class KinectPublisher : public rclcpp::Node {
public:
  KinectPublisher() : Node("kinect_publisher") {
    serial_ = declare_parameter<std::string>("serial", "");
    frame_prefix_ = declare_parameter<std::string>("frame_prefix", "kinect_front");
    packet_pipeline_ = declare_parameter<std::string>("packet_pipeline", "opengl");
    publish_color_ = declare_parameter<bool>("publish_color", true);
    publish_depth_ = declare_parameter<bool>("publish_depth", true);
    enable_registration_ = declare_parameter<bool>("enable_registration", false);
    if (enable_registration_ && (!publish_color_ || !publish_depth_)) {
      throw std::runtime_error("Registration requires both color and depth streams");
    }
    if (serial_.empty() || (!publish_color_ && !publish_depth_)) {
      throw std::runtime_error("Must supply serial and enable a stream");
    }
    // Node namespace is supplied by launch; relative topic names avoid collisions.
    if (publish_color_) {
      color_pub_ = create_publisher<sensor_msgs::msg::Image>("color/image_raw", rclcpp::SensorDataQoS());
      color_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>("color/camera_info", rclcpp::SensorDataQoS());
    }
    if (publish_depth_) {
      depth_pub_ = create_publisher<sensor_msgs::msg::Image>("depth/image_raw", rclcpp::SensorDataQoS());
      depth_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>("depth/camera_info", rclcpp::SensorDataQoS());
    }
    if (enable_registration_) {
      registered_color_pub_ = create_publisher<sensor_msgs::msg::Image>(
          "color_registered/image_raw", rclcpp::SensorDataQoS());
      // Trial: reliable registered depth for exact-timestamp pose matching.
      // Keep raw depth, registered RGB and other image publishers on SensorDataQoS.
      depth_rect_pub_ = create_publisher<sensor_msgs::msg::Image>(
          "depth_rect/image_raw", rclcpp::QoS(rclcpp::KeepLast(10)).reliable());
      depth_rect_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>(
          "depth_rect/camera_info", rclcpp::SensorDataQoS());
    }
    if (freenect_.enumerateDevices() < 1) {
      throw std::runtime_error("No Kinect v2 devices detected");
    }
    // Ownership transfers to libfreenect2::Freenect2::openDevice().
    // NEVER keep a unique_ptr or delete this pipeline in this application.
    libfreenect2::PacketPipeline *pipeline = nullptr;
    if (packet_pipeline_ == "cpu") {
      pipeline = new libfreenect2::CpuPacketPipeline();
    } else if (packet_pipeline_ == "opengl") {
#ifdef LIBFREENECT2_WITH_OPENGL_SUPPORT
      pipeline = new libfreenect2::OpenGLPacketPipeline();
#else
      throw std::runtime_error("libfreenect2 was not compiled with OpenGL support");
#endif
    } else {
      throw std::runtime_error("packet_pipeline must be 'cpu' or 'opengl'");
    }
    device_ = freenect_.openDevice(serial_, pipeline);
    if (!device_) {
      throw std::runtime_error("Cannot open Kinect serial " + serial_);
    }
    unsigned int types = 0;
    if (publish_color_) types |= libfreenect2::Frame::Color;
    if (publish_depth_) types |= libfreenect2::Frame::Depth;
    listener_ = std::make_unique<libfreenect2::SyncMultiFrameListener>(types);
    if (publish_color_) device_->setColorFrameListener(listener_.get());
    if (publish_depth_) device_->setIrAndDepthFrameListener(listener_.get());
    if (!device_->startStreams(publish_color_, publish_depth_)) {
      device_->close(); device_ = nullptr;
      throw std::runtime_error("Could not start Kinect streams");
    }
    // Intrinsics exposed by libfreenect2; RGB intrinsics are for native RGB frame.
    const auto rgb = device_->getColorCameraParams();
    const auto ir = device_->getIrCameraParams();
    if (publish_color_) {
      color_info_ = cameraInfo(1920, 1080, rgb.fx, rgb.fy, rgb.cx, rgb.cy, frame_prefix_ + "_color_optical_frame");
      // libfreenect2 color calibration is not a conventional plumb_bob distortion model.
      color_info_.distortion_model = "";
      color_info_.d.clear();
    }
    if (publish_depth_) {
      depth_info_ = cameraInfo(512, 424, ir.fx, ir.fy, ir.cx, ir.cy, frame_prefix_ + "_depth_optical_frame");
      depth_info_.distortion_model = "plumb_bob";
      depth_info_.d = {ir.k1, ir.k2, ir.p1, ir.p2, ir.k3};
    }
    if (enable_registration_) {
      registration_ = std::make_unique<libfreenect2::Registration>(ir, rgb);
      depth_rect_info_ = cameraInfo(512, 424, ir.fx, ir.fy, ir.cx, ir.cy,
                                    frame_prefix_ + "_depth_optical_frame");
      // The depth geometry has already been undistorted by Registration::apply().
      depth_rect_info_.distortion_model = "plumb_bob";
      depth_rect_info_.d = {0., 0., 0., 0., 0.};
    }
    RCLCPP_INFO(get_logger(), "Started Kinect %s using %s pipeline; native RGB/depth, registration=%s",
                serial_.c_str(), packet_pipeline_.c_str(), enable_registration_ ? "ON" : "OFF");
    worker_ = std::thread([this] { process(); });
  }

  ~KinectPublisher() override {
    running_ = false;
    // Some libfreenect2 builds use tinythread: waitForNewFrame timeout is ignored.
    // Our worker polls hasNewFrame(), allowing it to exit without waiting forever.
    if (worker_.joinable()) worker_.join();
    if (device_) { device_->stop(); device_->close(); device_ = nullptr; }
    // libfreenect2 owns the packet pipeline and deletes it with its device.
  }

private:
  static sensor_msgs::msg::CameraInfo cameraInfo(uint32_t w, uint32_t h, double fx, double fy, double cx, double cy, const std::string &frame) {
    sensor_msgs::msg::CameraInfo info;
    info.header.frame_id = frame;
    info.width = w; info.height = h;
    info.k = {fx, 0., cx, 0., fy, cy, 0., 0., 1.};
    info.r = {1., 0., 0., 0., 1., 0., 0., 0., 1.};
    info.p = {fx, 0., cx, 0., 0., fy, cy, 0., 0., 0., 1., 0.};
    return info;
  }
  void process() {
    // Reuse buffers instead of allocating large frames on each sample.
    libfreenect2::Frame undistorted(512, 424, 4);
    libfreenect2::Frame registered(512, 424, 4);
    uint64_t count = 0;
    auto last = std::chrono::steady_clock::now();
    while (running_ && rclcpp::ok()) {
      libfreenect2::FrameMap frames;
      if (!listener_->hasNewFrame()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        continue;
      }
      // With hasNewFrame() true, this returns immediately; do not rely on
      // the timeout overload when libfreenect2 was built with tinythread.
      listener_->waitForNewFrame(frames);
      // Host receive timestamp. It is NOT device-hardware synchronized between Kinect units.
      const auto stamp = now();
      if (publish_color_) {
        auto *f = frames[libfreenect2::Frame::Color];
        if (f && f->bytes_per_pixel == 4) {
          sensor_msgs::msg::Image im;
          im.header.stamp = stamp;
          im.header.frame_id = frame_prefix_ + "_color_optical_frame";
          im.height = f->height; im.width = f->width;
          im.encoding = (f->format == libfreenect2::Frame::RGBX) ? "rgba8" : "bgra8"; im.is_bigendian = false;
          im.step = f->width * 4;
          im.data.resize(static_cast<size_t>(im.step) * im.height);
          std::memcpy(im.data.data(), f->data, im.data.size());
          color_info_.header.stamp = stamp;
          color_pub_->publish(std::move(im));
          color_info_pub_->publish(color_info_);
        }
      }
      if (publish_depth_) {
        auto *f = frames[libfreenect2::Frame::Depth];
        if (f && f->bytes_per_pixel == 4) {
          sensor_msgs::msg::Image im;
          im.header.stamp = stamp;
          im.header.frame_id = frame_prefix_ + "_depth_optical_frame";
          im.height = f->height; im.width = f->width;
          im.encoding = "32FC1"; im.is_bigendian = false;
          im.step = f->width * sizeof(float);
          im.data.resize(static_cast<size_t>(im.step) * im.height);
          // libfreenect2 native depth stores float millimeters; publish ROS depth in meters.
          const float *src = reinterpret_cast<const float *>(f->data);
          float *dst = reinterpret_cast<float *>(im.data.data());
          const size_t n = static_cast<size_t>(im.width) * im.height;
          for (size_t i = 0; i < n; ++i) dst[i] = src[i] > 0.0f ? src[i] * 0.001f : std::numeric_limits<float>::quiet_NaN();
          depth_info_.header.stamp = stamp;
          depth_pub_->publish(std::move(im));
          depth_info_pub_->publish(depth_info_);
        }
      }
      if (enable_registration_ && registration_) {
        auto *rgb = frames[libfreenect2::Frame::Color];
        auto *depth = frames[libfreenect2::Frame::Depth];
        if (rgb && depth && rgb->bytes_per_pixel == 4 && depth->bytes_per_pixel == 4) {
          // Factory camera-to-camera registration. RGB and undistorted depth
          // share 512x424 pixel coordinates after this call.
          registration_->apply(rgb, depth, &undistorted, &registered);
          sensor_msgs::msg::Image ci;
          ci.header.stamp = stamp;
          ci.header.frame_id = frame_prefix_ + "_depth_optical_frame";
          ci.width = 512; ci.height = 424;
          ci.encoding = (rgb->format == libfreenect2::Frame::RGBX) ? "rgba8" : "bgra8";
          ci.is_bigendian = false;
          ci.step = 512 * 4;
          ci.data.resize(static_cast<size_t>(ci.step) * ci.height);
          std::memcpy(ci.data.data(), registered.data, ci.data.size());
          registered_color_pub_->publish(std::move(ci));

          sensor_msgs::msg::Image di;
          di.header.stamp = stamp;
          di.header.frame_id = frame_prefix_ + "_depth_optical_frame";
          di.width = 512; di.height = 424;
          di.encoding = "32FC1"; di.is_bigendian = false;
          di.step = 512 * sizeof(float);
          di.data.resize(static_cast<size_t>(di.step) * di.height);
          // libfreenect2 depth is measured in millimeters; ROS topic is meters.
          const float *src = reinterpret_cast<const float *>(undistorted.data);
          float *dst = reinterpret_cast<float *>(di.data.data());
          for (size_t i = 0; i < 512u * 424u; ++i) {
            dst[i] = std::isfinite(src[i]) && src[i] > 0.f
                         ? src[i] * 0.001f : std::numeric_limits<float>::quiet_NaN();
          }
          depth_rect_info_.header.stamp = stamp;
          depth_rect_pub_->publish(std::move(di));
          depth_rect_info_pub_->publish(depth_rect_info_);
        }
      }
      listener_->release(frames);
      ++count;
      auto t = std::chrono::steady_clock::now();
      if (t - last >= std::chrono::seconds(5)) {
        double sec = std::chrono::duration<double>(t - last).count();
        RCLCPP_INFO(get_logger(), "Kinect %s: %.1f received frame sets/s (host)", serial_.c_str(), count / sec);
        count = 0; last = t;
      }
    }
  }
  std::string serial_, frame_prefix_, packet_pipeline_;
  bool publish_color_, publish_depth_, enable_registration_;
  std::atomic<bool> running_{true};
  libfreenect2::Freenect2 freenect_;
  libfreenect2::Freenect2Device *device_ = nullptr;
  std::unique_ptr<libfreenect2::SyncMultiFrameListener> listener_;
  std::unique_ptr<libfreenect2::Registration> registration_;
  std::thread worker_;
  sensor_msgs::msg::CameraInfo color_info_, depth_info_, depth_rect_info_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr color_pub_, depth_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr registered_color_pub_, depth_rect_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr color_info_pub_, depth_info_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr depth_rect_info_pub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<KinectPublisher>();
    rclcpp::spin(node);
  } catch (const std::exception &e) {
    RCLCPP_FATAL(rclcpp::get_logger("kinect_v2_ros2_driver"), "%s", e.what());
    if (rclcpp::ok()) rclcpp::shutdown();
    return 1;
  }
  if (rclcpp::ok()) rclcpp::shutdown();
  return 0;
}
