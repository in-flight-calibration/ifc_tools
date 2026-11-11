#pragma once

#include <ifc_tools/ring_buffer.hpp>

#include <rclcpp/rclcpp.hpp>

#include <atomic>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace ifc_tools
{

template <typename Message, typename T>
class MessageConverter
{
public:
    virtual ~MessageConverter() = default;

    virtual std::optional<T> convert(const Message& message) const = 0;
};

template <typename Message, typename T>
class BufferedReceiver : public rclcpp::Node
{
public:
    BufferedReceiver(
        const std::string& node_name,
        std::shared_ptr<const MessageConverter<Message, T>> converter,
        std::size_t capacity,
        const std::string& topic,
        const rclcpp::QoS& qos = rclcpp::SensorDataQoS())
        : rclcpp::Node(node_name), _converter(std::move(converter)), _buffer(capacity)
    {
        _subscription = create_subscription<Message>(
            topic, qos,
            [this](const typename Message::ConstSharedPtr message)
            {
                receive(*message);
            });
    }

    std::vector<T> snapshot()
    {
        std::scoped_lock<std::mutex> lock(_buffer_mtx);
        return _buffer.toVector();
    }

private:
    std::shared_ptr<const MessageConverter<Message, T>> _converter;

    RingBuffer<T> _buffer;
    std::mutex _buffer_mtx;
    
    typename rclcpp::Subscription<Message>::SharedPtr _subscription;

    void receive(const Message& message)
    {
        std::optional<T> sample = _converter->convert(message);

        if (!sample)
        {
            return;
        }

        std::scoped_lock<std::mutex> lock(_buffer_mtx);

        if (!_buffer.empty())
        {
            const auto last = _buffer.back().timestamp;

            if (sample->timestamp == last)
            {
                return;
            }

            if (sample->timestamp < last)
            {
                _buffer.clear();
                RCLCPP_WARN(get_logger(), "Timestamp went backwards, buffer cleared");
            }
        }

        _buffer.push(std::move(*sample));
    }
};

}
