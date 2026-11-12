#pragma once

#include <ifc_tools/processing/interpolator.hpp>
#include <ifc_tools/receiver/buffered_receiver.hpp>

#include <px4_msgs/msg/vehicle_local_position.hpp>

#include <Eigen/Core>

#include <cstdint>
#include <limits>
#include <optional>

namespace ifc_tools
{

struct VehicleLocalPosition
{
    int64_t timestamp = 0;
    Eigen::Vector3d position = Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
    Eigen::Vector3d velocity = Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
};

using VehicleLocalPositionReceiver = BufferedReceiver<px4_msgs::msg::VehicleLocalPosition, VehicleLocalPosition>;

class VehicleLocalPositionUtils
    : public MessageConverter<px4_msgs::msg::VehicleLocalPosition, VehicleLocalPosition>,
      public InterpolationMethod<VehicleLocalPosition>
{
public:
    std::optional<VehicleLocalPosition> convert(const px4_msgs::msg::VehicleLocalPosition& message) const override
    {
        VehicleLocalPosition result;
        result.timestamp = static_cast<int64_t>(message.timestamp_sample) * 1000;

        if (message.xy_valid)
        {
            result.position.x() = message.x;
            result.position.y() = message.y;
        }

        if (message.z_valid)
        {
            result.position.z() = message.z;
        }

        if (message.v_xy_valid)
        {
            result.velocity.x() = message.vx;
            result.velocity.y() = message.vy;
        }

        if (message.v_z_valid)
        {
            result.velocity.z() = message.vz;
        }

        return result;
    }

    VehicleLocalPosition interpolate(const VehicleLocalPosition& a, const VehicleLocalPosition& b, float alpha) const override
    {
        const double t = static_cast<double>(alpha);
        return {a.timestamp, a.position + t * (b.position - a.position), a.velocity + t * (b.velocity - a.velocity)};
    }
};

}
