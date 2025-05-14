#include <memory>
#include <fstream>
#include <sstream>
#include <vector>
#include <map>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "builtin_interfaces/msg/time.hpp"
#include "rclcpp_components/register_node_macro.hpp"

using std::placeholders::_1;

namespace latsub_ns
{

class LatencySubscriberComponent : public rclcpp::Node
{
public:
  explicit LatencySubscriberComponent(const rclcpp::NodeOptions & options)
  : Node("message_subscriber", options)
  {
    RCLCPP_INFO(this->get_logger(), "ROS2 params setting initiate!");

    // 기본 파라미터
    std::map<std::string, rclcpp::ParameterValue> defaults = {
      {"payload_size", rclcpp::ParameterValue(1024)},
      {"period_hz",    rclcpp::ParameterValue(100.0)},
      {"sample_count", rclcpp::ParameterValue(10000)},
      {"qos_depth",    rclcpp::ParameterValue(10)},
      {"qos_reliable", rclcpp::ParameterValue(true)},
      {"qos_history",  rclcpp::ParameterValue(std::string("KEEP_LAST"))}
    };
    for (auto &kv : defaults) {
      this->declare_parameter(kv.first, kv.second);
    }

    payload_size_ = this->get_parameter("payload_size").as_int();
    period_hz_    = this->get_parameter("period_hz").as_double();
    sample_count_ = this->get_parameter("sample_count").as_int();
    qos_depth_    = this->get_parameter("qos_depth").as_int();
    qos_reliable_ = this->get_parameter("qos_reliable").as_bool();
    qos_history_  = this->get_parameter("qos_history").as_string();

    received_msgs_.reserve(sample_count_);
    received_times_.reserve(sample_count_);

    RCLCPP_INFO(this->get_logger(), "payload_size = %d", payload_size_);
    RCLCPP_INFO(this->get_logger(), "period_hz    = %.2f", period_hz_);
    RCLCPP_INFO(this->get_logger(), "sample_count = %d", sample_count_);
    RCLCPP_INFO(this->get_logger(), "qos_depth    = %d", qos_depth_);
    RCLCPP_INFO(this->get_logger(), "qos_reliable = %s", qos_reliable_ ? "true":"false");
    RCLCPP_INFO(this->get_logger(), "qos_history  = %s", qos_history_.c_str());

    // QoS 프로필
    rclcpp::QoS qos(qos_depth_);
    qos = qos_reliable_ ? qos.reliable() : qos.best_effort();
    if (qos_history_ == "KEEP_LAST") {
      qos = qos.history(RMW_QOS_POLICY_HISTORY_KEEP_LAST).keep_last(qos_depth_);
    } else {
      qos = qos.history(RMW_QOS_POLICY_HISTORY_KEEP_ALL);
    }

    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
      "topic", qos,
      std::bind(&LatencySubscriberComponent::topic_callback, this, _1));
  }

  ~LatencySubscriberComponent()
  {
    std::stringstream fn;
    fn << "sub_log"
       << "_size"  << payload_size_
       << "_hz"    << static_cast<int>(period_hz_)
       << "_count"<< sample_count_
       << "_qos"   << qos_history_ << "_" << (qos_reliable_?"RELIABLE":"BEST_EFFORT")
       << ".csv";
    std::ofstream ofs(fn.str());
    ofs << "seq,t_sent_ns,t_recv_ns\n";
    for (size_t i = 0; i < received_msgs_.size(); ++i) {
      auto &hdr = received_msgs_[i];
      int64_t t_sent = hdr.sec * 1000000000LL + hdr.nanosec;
      int64_t t_recv = received_times_[i].nanoseconds();
      ofs << i << "," << t_sent << "," << t_recv << "\n";
    }
    RCLCPP_INFO(this->get_logger(), "Saved %zu msgs to %s",
                received_msgs_.size(), fn.str().c_str());
  }

private:
  void topic_callback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    received_msgs_.push_back(msg->header.stamp);
    received_times_.push_back(this->now());
  }

  int    payload_size_;  
  double period_hz_;     
  int    sample_count_;  
  int    qos_depth_;     
  bool   qos_reliable_;  
  std::string qos_history_;

  std::vector<builtin_interfaces::msg::Time> received_msgs_;
  std::vector<rclcpp::Time>                  received_times_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
};

RCLCPP_COMPONENTS_REGISTER_NODE(latsub_ns::LatencySubscriberComponent)

}  // namespace latsub_ns

