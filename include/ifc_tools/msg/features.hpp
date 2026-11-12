#pragma once

#include <ifc_tools/receiver/buffered_receiver.hpp>

#include <ifc_msgs/msg/features.hpp>

#include <rclcpp/time.hpp>

#include <Eigen/Core>

#include <array>
#include <cmath>
#include <cstdint>
#include <optional>

namespace ifc_tools
{

struct Features
{
    std::array<int64_t, 2> timestamp{0, 0};
    Eigen::Matrix2d A = Eigen::Matrix2d::Identity();
    Eigen::Vector2d b = Eigen::Vector2d::Zero();
};

using FeaturesReceiver = BufferedReceiver<ifc_msgs::msg::Features, Features>;

class FeaturesUtils : public MessageConverter<ifc_msgs::msg::Features, Features>
{
public:
    std::optional<Features> convert(const ifc_msgs::msg::Features& message) const override
    {
        for (const float value : message.affine)
        {
            if (!std::isfinite(value))
            {
                return std::nullopt;
            }
        }

        Features features;
        features.timestamp = {
            rclcpp::Time(message.image_timestamp[0]).nanoseconds(),
            rclcpp::Time(message.image_timestamp[1]).nanoseconds()};
        features.A << message.affine[0], message.affine[1], message.affine[3], message.affine[4];
        features.b << message.affine[2], message.affine[5];

        if (features.timestamp[1] <= features.timestamp[0])
        {
            return std::nullopt;
        }

        return features;
    }
};

}
