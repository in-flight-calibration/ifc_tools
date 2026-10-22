#pragma once

#include <cv_bridge/cv_bridge.hpp>
#include <image_transport/image_transport.hpp>
#include <opencv2/core/mat.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <functional>
#include <mutex>
#include <optional>
#include <string>
#include <utility>

namespace ifc_tools
{

class ImageReceiver : public rclcpp::Node
{
public:
    using Callback = std::function<void(const rclcpp::Time&, const cv::Mat&)>;

    ImageReceiver(
        const std::string& node_name,
        const std::string& topic,
        const std::string& transport)
        : rclcpp::Node(node_name)
    {
        _subscriber = image_transport::create_subscription(
            *this, topic,
            [this](const sensor_msgs::msg::Image::ConstSharedPtr& message)
            {
                receiveImage(message);
            },
            transport, rclcpp::SensorDataQoS());
    }

    void setCallback(Callback callback)
    {
        std::lock_guard<std::mutex> lock(_callback_mutex);

        if (callback)
        {
            _callback = std::move(callback);
        }
        else
        {
            _callback.reset();
        }
    }

private:
    void receiveImage(const sensor_msgs::msg::Image::ConstSharedPtr& message)
    {
        cv_bridge::CvImageConstPtr image;

        try
        {
            image = cv_bridge::toCvShare(message, "bgr8");
        }
        catch (const cv_bridge::Exception& error)
        {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 5000,
                "Failed to decode image: %s", error.what());
            return;
        }

        std::lock_guard<std::mutex> lock(_callback_mutex);

        if (!_callback)
        {
            return;
        }

        (*_callback)(rclcpp::Time(message->header.stamp, get_clock()->get_clock_type()), image->image);
    }

    std::mutex _callback_mutex;
    std::optional<Callback> _callback;
    image_transport::Subscriber _subscriber;
};

}
