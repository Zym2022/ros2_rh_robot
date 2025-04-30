#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <memory>
#include <string>
#include <chrono>
#include <thread>

class CustomTfBroadcaster : public rclcpp::Node
{
public:
  CustomTfBroadcaster()
  : Node("custom_tf_broadcaster"), transform_available_(false)
  {
    // 获取参数
    this->declare_parameter<std::string>("source_frame", "odom");       // 用于监听的源坐标系
    this->declare_parameter<std::string>("target_frame", "base_footprint"); // 用于监听的目标坐标系
    this->declare_parameter<std::string>("new_source_frame", "world");  // 新的发布源坐标系
    this->declare_parameter<std::string>("new_target_frame", "base_link"); // 新的发布目标坐标系
    this->declare_parameter<double>("publish_frequency", 100.0);
    this->declare_parameter<double>("wait_timeout", 30.0); // 等待变换的超时时间（秒）
    
    source_frame_ = this->get_parameter("source_frame").as_string();
    target_frame_ = this->get_parameter("target_frame").as_string();
    new_source_frame_ = this->get_parameter("new_source_frame").as_string();
    new_target_frame_ = this->get_parameter("new_target_frame").as_string();
    double wait_timeout = this->get_parameter("wait_timeout").as_double();

    // 创建TF广播器
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    
    // 创建TF缓冲区和监听器
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
    
    RCLCPP_INFO(this->get_logger(), "自定义TF广播器已启动");
    RCLCPP_INFO(this->get_logger(), "将监听变换: %s -> %s, 并广播变换: %s -> %s",
                source_frame_.c_str(), target_frame_.c_str(),
                new_source_frame_.c_str(), new_target_frame_.c_str());
    
    // 等待变换可用
    RCLCPP_INFO(this->get_logger(), "正在等待变换 %s -> %s 发布...", 
                source_frame_.c_str(), target_frame_.c_str());
    
    // 创建检查变换的定时器（更短的周期）
    check_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500), // 每500毫秒检查一次
      std::bind(&CustomTfBroadcaster::check_transform_available, this));
    
    // 等待变换可用的超时定时器
    timeout_timer_ = this->create_wall_timer(
      std::chrono::duration<double>(wait_timeout),
      [this]() {
        if (!transform_available_) {
          RCLCPP_WARN(this->get_logger(), 
                     "等待变换 %s -> %s 超时，仍将继续尝试获取变换...", 
                     source_frame_.c_str(), target_frame_.c_str());
        }
        timeout_timer_->cancel(); // 只触发一次
      });
  }

private:
  void check_transform_available()
  {
    try {
      // 尝试查询变换
      auto transform = tf_buffer_->lookupTransform(
        source_frame_, target_frame_, tf2::TimePointZero);
      
      // 如果成功获取变换且之前未标记为可用
      if (!transform_available_) {
        transform_available_ = true;
        RCLCPP_INFO(this->get_logger(), "变换 %s -> %s 已可用，开始广播 %s -> %s",
                    source_frame_.c_str(), target_frame_.c_str(),
                    new_source_frame_.c_str(), new_target_frame_.c_str());
        
        // 创建定时广播变换的定时器
        double publish_frequency = this->get_parameter("publish_frequency").as_double();
        auto period = std::chrono::duration<double>(1.0 / publish_frequency);
        publish_timer_ = this->create_wall_timer(
          std::chrono::duration_cast<std::chrono::milliseconds>(period),
          std::bind(&CustomTfBroadcaster::publish_tf, this));
        
        // 取消检查定时器
        check_timer_->cancel();
      }
    } catch (const tf2::TransformException &ex) {
      // 在等待阶段不输出错误，只有在超时后才输出
      if (transform_available_) {
        RCLCPP_WARN(this->get_logger(), "无法获取变换: %s", ex.what());
      }
    }
  }

  void publish_tf()
  {
    try {
      // 查找指定的变换 (odom -> base_footprint)
      geometry_msgs::msg::TransformStamped transform_stamped;
      transform_stamped = tf_buffer_->lookupTransform(
        source_frame_, target_frame_, tf2::TimePointZero);

      // 使用当前系统时间创建新的变换 (world -> base_link)
      geometry_msgs::msg::TransformStamped new_transform;
      new_transform.header.stamp = this->now(); // 使用系统实时时间
      new_transform.header.frame_id = new_source_frame_; // world
      new_transform.child_frame_id = new_target_frame_; // base_link
      
      // 复制变换数据
      new_transform.transform = transform_stamped.transform;
      
      // 广播新的变换
      tf_broadcaster_->sendTransform(new_transform);
    } catch (const tf2::TransformException &ex) {
      RCLCPP_WARN(this->get_logger(), "无法获取变换: %s", ex.what());
    }
  }

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::TimerBase::SharedPtr check_timer_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
  rclcpp::TimerBase::SharedPtr timeout_timer_;
  std::string source_frame_;
  std::string target_frame_;
  std::string new_source_frame_;
  std::string new_target_frame_;
  bool transform_available_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CustomTfBroadcaster>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
} 