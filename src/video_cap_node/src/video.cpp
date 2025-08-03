#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <chrono>
#include <memory>

using namespace std::chrono_literals;

class ImageSubscriber : public rclcpp::Node
{
public:
    ImageSubscriber() : Node("image_to_video_node")
    {
        // 初始化参数
        output_file_ = "output.mp4";
        fps_ = 30.0;  // 默认帧率
        frame_counter_ = 0;
        first_frame_received_ = false;

        // 创建订阅者
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/armor_detector/result_img",
            10,
            std::bind(&ImageSubscriber::image_callback, this, std::placeholders::_1));

        // 初始化计时器
        timer_ = this->create_wall_timer(
            1000ms, std::bind(&ImageSubscriber::timer_callback, this));
    }

    // 添加公共访问器函数
    size_t get_frame_counter() const {
        return frame_counter_;
    }

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        try {
            // 将ROS图像消息转换为OpenCV格式
            cv::Mat frame = cv_bridge::toCvCopy(msg, "bgr8")->image;
            
            if (!first_frame_received_) {
                // 初始化视频写入器
                int width = frame.cols;
                int height = frame.rows;
                video_writer_.open(
                    output_file_,
                    cv::VideoWriter::fourcc('m', 'p', '4', 'v'),
                    fps_,
                    cv::Size(width, height)
                );
                
                if (!video_writer_.isOpened()) {
                    RCLCPP_ERROR(this->get_logger(), "Could not open the video file for writing");
                    return;
                }
                
                RCLCPP_INFO(this->get_logger(), "Started recording video to %s", output_file_.c_str());
                first_frame_received_ = true;
            }
            
            // 写入帧到视频
            video_writer_.write(frame);
            frame_counter_++;
            
        } catch (const cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        }
    }

    void timer_callback()
    {
        if (first_frame_received_) {
            RCLCPP_INFO(this->get_logger(), "Recording - Frames saved: %d", frame_counter_);
        } else {
            RCLCPP_WARN(this->get_logger(), "Waiting for first image frame...");
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    rclcpp::TimerBase::SharedPtr timer_;
    cv::VideoWriter video_writer_;
    std::string output_file_;
    double fps_;
    size_t frame_counter_;
    bool first_frame_received_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ImageSubscriber>();
    rclcpp::spin(node);
    
    // 使用公共访问器函数获取帧计数器
    RCLCPP_INFO(node->get_logger(), "Stopping recording. Total frames saved: %zu", node->get_frame_counter());
    rclcpp::shutdown();
    return 0;
}