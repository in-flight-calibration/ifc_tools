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
    double ref_lat = std::numeric_limits<double>::quiet_NaN();
    double ref_lon = std::numeric_limits<double>::quiet_NaN();
    float ref_alt = std::numeric_limits<float>::quiet_NaN();
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

        if (message.xy_global)
        {
            result.ref_lat = message.ref_lat;
            result.ref_lon = message.ref_lon;
        }

        if (message.z_global)
        {
            result.ref_alt = message.ref_alt;
        }

        return result;
    }

    VehicleLocalPosition interpolate(const VehicleLocalPosition& a, const VehicleLocalPosition& b, float alpha) const override
    {
        const double t = static_cast<double>(alpha);

        VehicleLocalPosition result;
        result.timestamp = a.timestamp;
        result.position = a.position + t * (b.position - a.position);
        result.velocity = a.velocity + t * (b.velocity - a.velocity);

        if (a.ref_lat == b.ref_lat && a.ref_lon == b.ref_lon && a.ref_alt == b.ref_alt)
        {
            result.ref_lat = a.ref_lat;
            result.ref_lon = a.ref_lon;
            result.ref_alt = a.ref_alt;
        }

        return result;
    }
};

}
