#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp" 

#include "rclcpp/executors/multi_threaded_executor.hpp" // for multithread
using namespace std::chrono_literals; // for time literal convinience

class MessagePublisher : public rclcpp::Node
{
  public:
    MessagePublisher()
    : Node("message_publisher")
    {
      RCLCPP_INFO(this->get_logger(), "ROS2 params setting initiate!");

      std::map<std::string, rclcpp::ParameterValue> param_defaults=
      {
        {"payload_size", rclcpp::ParameterValue(1024)},
        {"period_hz", rclcpp::ParameterValue(100.0)},
        {"sample_count", rclcpp::ParameterValue(10000)},
        {"qos_depth", rclcpp::ParameterValue(10)},
        {"qos_reliable", rclcpp::ParameterValue(true)},
        {"qos_history", rclcpp::ParameterValue(std::string("KEEP_LAST"))}
      };

      RCLCPP_INFO(this->get_logger(), "ROS2 Declare params!");
      //Node::declare_parameter()
      this->declare_parameter("payload_size", param_defaults["payload_size"]);
      this->declare_parameter("period_hz", param_defaults["period_hz"]);
      this->declare_parameter("sample_count", param_defaults["sample_count"]);
      this->declare_parameter("qos_depth", param_defaults["qos_depth"]);
      this->declare_parameter("qos_reliable", param_defaults["qos_reliable"]);
      this->declare_parameter("qos_history", param_defaults["qos_history"]);

      payload_size_ = this->get_parameter("payload_size").as_int();
      period_hz_ = this->get_parameter("period_hz").as_double();
      sample_count_ = this->get_parameter("sample_count").as_int();
      qos_depth_ = this->get_parameter("qos_depth").as_int();
      qos_reliable_ = this->get_parameter("qos_reliable").as_bool();
      qos_history_ = this->get_parameter("qos_history").as_string();

      //Debug
      RCLCPP_INFO(this->get_logger(), "payload_size = %d", payload_size_);
      RCLCPP_INFO(this->get_logger(), "period_hz = %.2f", period_hz_);
      RCLCPP_INFO(this->get_logger(), "sample_count = %d", sample_count_);
      RCLCPP_INFO(this->get_logger(), "qos_depth = %d", qos_depth_);
      RCLCPP_INFO(this->get_logger(), "qos_reliable = %s", qos_reliable_ ? "true" : "false");
      RCLCPP_INFO(this->get_logger(), "qos_history = %s", qos_history_.c_str());

      //Qos setting
      rclcpp::QoS qos_profile(qos_depth_);
      if(qos_reliable_){
        qos_profile = qos_profile.reliable();
      }
      else{
        qos_profile = qos_profile.best_effort();
      }

      if(qos_history_ == "KEEP_LAST"){
        qos_profile = qos_profile.history(RMW_QOS_POLICY_HISTORY_KEEP_LAST);
        qos_profile = qos_profile.keep_last(qos_depth_);
      }
      else{
        qos_profile = qos_profile.history(RMW_QOS_POLICY_HISTORY_KEEP_ALL);
      }

      //image message format
      msg.height = 1;
      msg.width = payload_size_;
      msg.encoding = "mono8"; //grey 8bit
      msg.is_bigendian = false; // bigendian is only used on network layer
      msg.step = payload_size_; // byte per line
      msg.data = std::vector<uint8_t>(payload_size_, 0xAB);  

      //ros2 publisher creation 
      publisher_ = this->create_publisher<sensor_msgs::msg::Image>("topic", qos_profile);
      timer_ = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(1000.0 / period_hz_)),
        std::bind(&MessagePublisher::timer_callback, this)
      );
    }

  private:

    sensor_msgs::msg::Image msg;
    int payload_size_;
    double period_hz_;
    int sample_count_;
    int qos_depth_;
    bool qos_reliable_;
    std::string qos_history_;
    int count_ = 0;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;

    void timer_callback()
    {
      if (count_ >= sample_count_) {
        RCLCPP_INFO(this->get_logger(), "%d / %d Msg publish completed. terminate node.", count_, sample_count_);
        rclcpp::shutdown();
        return;
      }

      //header format
      //msg.header.seq = count_++; //msg seq
      msg.header.stamp = this->now(); //timestamp for now
      msg.header.frame_id = "custom_frame"; // for rviz and analyze
      publisher_->publish(msg);
      ++count_;
      //callback loop optimize - delete debug code
    }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MessagePublisher>();
  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}
