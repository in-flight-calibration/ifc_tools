#pragma once

#include <ifc_msgs/msg/performance.hpp>
#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <limits>
#include <mutex>
#include <optional>
#include <string>

namespace ifc_tools
{

class PerformanceMonitor : public rclcpp::Node
{
public:
    explicit PerformanceMonitor(const std::string& name)
        : rclcpp::Node(name + "_performance"),
          _name(name)
    {
    }

    void setTopic(const std::string& topic)
    {
        std::lock_guard<std::mutex> lock(_mutex);

        _publisher = create_publisher<ifc_msgs::msg::Performance>(topic, rclcpp::SensorDataQoS());
        _window_start = std::chrono::steady_clock::now();
        _timer = create_wall_timer(std::chrono::seconds(1), [this]() { publish(); });
    }

    void start()
    {
        std::lock_guard<std::mutex> lock(_mutex);
        _start_time = std::chrono::steady_clock::now();
    }

    void stop()
    {
        std::lock_guard<std::mutex> lock(_mutex);

        if (!_start_time)
        {
            return;
        }

        const int64_t duration_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now() - *_start_time).count();
        _start_time.reset();

        _count++;
        _sum_ns += duration_ns;
        _min_ns = std::min(_min_ns, duration_ns);
        _max_ns = std::max(_max_ns, duration_ns);
    }

private:
    void publish()
    {
        std::lock_guard<std::mutex> lock(_mutex);

        const auto now = std::chrono::steady_clock::now();
        const double elapsed = std::chrono::duration<double>(now - _window_start).count();

        ifc_msgs::msg::Performance message;
        message.header.stamp = get_clock()->now();
        message.name = _name;
        message.rate = static_cast<float>(_count / elapsed);
        message.min_ns = _count ? _min_ns : 0;
        message.avg_ns = _count ? _sum_ns / static_cast<int64_t>(_count) : 0;
        message.max_ns = _count ? _max_ns : 0;
        _publisher->publish(message);

        _count = 0;
        _sum_ns = 0;
        _min_ns = std::numeric_limits<int64_t>::max();
        _max_ns = 0;
        _window_start = now;
    }

    std::string _name;
    std::mutex _mutex;
    rclcpp::Publisher<ifc_msgs::msg::Performance>::SharedPtr _publisher;
    rclcpp::TimerBase::SharedPtr _timer;
    std::optional<std::chrono::steady_clock::time_point> _start_time;
    std::chrono::steady_clock::time_point _window_start;
    uint64_t _count = 0;
    int64_t _sum_ns = 0;
    int64_t _min_ns = std::numeric_limits<int64_t>::max();
    int64_t _max_ns = 0;
};

}
