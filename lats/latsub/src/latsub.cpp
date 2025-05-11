#include <memory>
#include <fstream>
#include <sstream>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "builtin_interfaces/msg/time.hpp" // Time 타입 사용

using std::placeholders::_1;
using namespace std::chrono_literals;

class MessageSubscriber : public rclcpp::Node
{
public:
    MessageSubscriber()
        : Node("message_subscriber")
    {
        RCLCPP_INFO(this->get_logger(), "ROS2 params setting initiate!");
        std::map<std::string, rclcpp::ParameterValue> param_defaults = {
            {"payload_size", rclcpp::ParameterValue(1024)},
            {"period_hz", rclcpp::ParameterValue(100.0)},
            {"sample_count", rclcpp::ParameterValue(10000)},
            {"qos_depth", rclcpp::ParameterValue(10)},
            {"qos_reliable", rclcpp::ParameterValue(true)},
            {"qos_history", rclcpp::ParameterValue(std::string("KEEP_LAST"))}};

        RCLCPP_INFO(this->get_logger(), "ROS2 Declare params!");
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
        
        received_msgs_.reserve(sample_count_);
        received_times_.reserve(sample_count_);

        RCLCPP_INFO(this->get_logger(), "payload_size = %d", payload_size_);
        RCLCPP_INFO(this->get_logger(), "period_hz = %.2f", period_hz_);
        RCLCPP_INFO(this->get_logger(), "sample_count = %d", sample_count_);
        RCLCPP_INFO(this->get_logger(), "qos_depth = %d", qos_depth_);
        RCLCPP_INFO(this->get_logger(), "qos_reliable = %s", qos_reliable_ ? "true" : "false");
        RCLCPP_INFO(this->get_logger(), "qos_history = %s", qos_history_.c_str());

        rclcpp::QoS qos_profile(qos_depth_);
        if (qos_reliable_)
        {
            qos_profile = qos_profile.reliable();
        }
        else
        {
            qos_profile = qos_profile.best_effort();
        }

        if (qos_history_ == "KEEP_LAST")
        {
            qos_profile = qos_profile.history(RMW_QOS_POLICY_HISTORY_KEEP_LAST);
            qos_profile = qos_profile.keep_last(qos_depth_);
        }
        else
        {
            qos_profile = qos_profile.history(RMW_QOS_POLICY_HISTORY_KEEP_ALL);
        }

        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "topic", qos_profile,
            std::bind(&MessageSubscriber::topic_callback, this, std::placeholders::_1));
    }

    ~MessageSubscriber()
    {
        std::string qos_mode = qos_reliable_ ? "RELIABLE" : "BEST_EFFORT";
        std::stringstream filename;
        filename << "sub_log"
                 << "_size" << payload_size_
                 << "_hz" << static_cast<int>(period_hz_)
                 << "_count" << sample_count_
                 << "_qos" << qos_history_ << "_" << qos_mode
                 << ".csv";

        std::ofstream log_file(filename.str());
        log_file << "seq,t_sent_ns,t_recv_ns\n"; // width, height, encoding은 일단 제외

        int seq = 0;
        for (size_t i = 0; i < received_msgs_.size(); ++i)
        {
            const auto &msg = received_msgs_[i];
            const auto &recv_time = received_times_[i];

            int64_t t_sent = static_cast<int64_t>(msg.sec) * 1'000'000'000LL + msg.nanosec;
            int64_t t_recv = static_cast<int64_t>(recv_time.nanoseconds());

            log_file << seq++ << "," << t_sent << "," << t_recv << "\n";
        }

        RCLCPP_INFO(this->get_logger(), "Saved %lu messages to %s", received_msgs_.size(), filename.str().c_str());
    }

private:
    int payload_size_;
    double period_hz_;
    int sample_count_;
    int qos_depth_;
    bool qos_reliable_;
    std::string qos_history_;
    std::vector<builtin_interfaces::msg::Time> received_msgs_; // stamp만 저장
    std::vector<rclcpp::Time> received_times_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;

    void topic_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        received_msgs_.push_back(msg->header.stamp); // stamp만 저장
        received_times_.push_back(this->now());
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MessageSubscriber>());
    rclcpp::shutdown();
    return 0;
}