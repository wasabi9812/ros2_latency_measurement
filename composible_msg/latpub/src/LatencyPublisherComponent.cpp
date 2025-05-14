// src/LatencyPublisherComponent.cpp

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <map>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "rclcpp_components/register_node_macro.hpp"

using namespace std::chrono_literals;

namespace latpub_ns
{

class LatencyPublisherComponent : public rclcpp::Node
{
public:
  explicit LatencyPublisherComponent(const rclcpp::NodeOptions & options)
  : Node("message_publisher", options)
  {
    RCLCPP_INFO(this->get_logger(), "ROS2 params setting initiate!");

    // 기본 파라미터 값들
    std::map<std::string, rclcpp::ParameterValue> param_defaults = {
      {"payload_size", rclcpp::ParameterValue(1024)},
      {"period_hz",      rclcpp::ParameterValue(100.0)},
      {"sample_count",   rclcpp::ParameterValue(10000)},
      {"qos_depth",      rclcpp::ParameterValue(10)},
      {"qos_reliable",   rclcpp::ParameterValue(true)},
      {"qos_history",    rclcpp::ParameterValue(std::string("KEEP_LAST"))}
    };

    // 파라미터 선언
    for (auto & kv : param_defaults) {
      this->declare_parameter(kv.first, kv.second);
    }

    // 파라미터 획득
    payload_size_ = this->get_parameter("payload_size").as_int();
    period_hz_    = this->get_parameter("period_hz").as_double();
    sample_count_ = this->get_parameter("sample_count").as_int();
    qos_depth_    = this->get_parameter("qos_depth").as_int();
    qos_reliable_ = this->get_parameter("qos_reliable").as_bool();
    qos_history_  = this->get_parameter("qos_history").as_string();

    // Debug 출력
    RCLCPP_INFO(this->get_logger(), "payload_size = %d", payload_size_);
    RCLCPP_INFO(this->get_logger(), "period_hz    = %.2f", period_hz_);
    RCLCPP_INFO(this->get_logger(), "sample_count = %d", sample_count_);
    RCLCPP_INFO(this->get_logger(), "qos_depth    = %d", qos_depth_);
    RCLCPP_INFO(this->get_logger(), "qos_reliable = %s",
                qos_reliable_ ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "qos_history  = %s", qos_history_.c_str());

    // QoS 프로필 구성
    rclcpp::QoS qos_profile(qos_depth_);
    qos_profile = qos_reliable_ ? qos_profile.reliable()
                                : qos_profile.best_effort();
    if (qos_history_ == "KEEP_LAST") {
      qos_profile = qos_profile.history(RMW_QOS_POLICY_HISTORY_KEEP_LAST)
                                .keep_last(qos_depth_);
    } else {
      qos_profile = qos_profile.history(RMW_QOS_POLICY_HISTORY_KEEP_ALL);
    }

    // Image 메시지 초기 설정
    msg_.height      = 1;
    msg_.width       = payload_size_;
    msg_.encoding    = "mono8";
    msg_.is_bigendian= false;
    msg_.step        = payload_size_;
    msg_.data        = std::vector<uint8_t>(payload_size_, 0xAB);

    // 퍼블리셔 & 타이머
    publisher_ = this->create_publisher<sensor_msgs::msg::Image>("topic", qos_profile);

    auto period_ms = std::chrono::milliseconds(static_cast<int>(1000.0 / period_hz_));
    timer_ = this->create_wall_timer(
      period_ms,
      std::bind(&LatencyPublisherComponent::timer_callback, this)
    );
  }

private:
  void timer_callback()
  {
    if (count_ >= sample_count_) {
      RCLCPP_INFO(this->get_logger(),
        "%d / %d Msg publish completed. terminating node.",
        count_, sample_count_);
      rclcpp::shutdown();
      return;
    }

    // 헤더에 timestamp, seq 삽입
    msg_.header.stamp = this->now();
    msg_.header.frame_id = "custom_frame";
    // msg_.header.seq는 자동 증가하지 않으므로 필요시 직접 관리하세요

    publisher_->publish(msg_);
    ++count_;
  }

  // 멤버 변수
  sensor_msgs::msg::Image msg_;
  int    payload_size_;
  double period_hz_;
  int    sample_count_;
  int    qos_depth_;
  bool   qos_reliable_;
  std::string qos_history_;
  int    count_ = 0;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
};

// 컴포넌트 등록
RCLCPP_COMPONENTS_REGISTER_NODE(latpub_ns::LatencyPublisherComponent)

}  // namespace latpub_ns

