#pragma once

#include <ifc_tools/processing/interpolator.hpp>
#include <ifc_tools/receiver/buffered_receiver.hpp>

#include <px4_msgs/msg/vehicle_global_position.hpp>

#include <cstdint>
#include <limits>
#include <optional>

namespace ifc_tools
{

struct VehicleGlobalPosition
{
    int64_t timestamp = 0;
    double latitude = std::numeric_limits<double>::quiet_NaN();
    double longitude = std::numeric_limits<double>::quiet_NaN();
    float altitude_amsl = std::numeric_limits<float>::quiet_NaN();
};

using VehicleGlobalPositionReceiver = BufferedReceiver<px4_msgs::msg::VehicleGlobalPosition, VehicleGlobalPosition>;

class VehicleGlobalPositionUtils
    : public MessageConverter<px4_msgs::msg::VehicleGlobalPosition, VehicleGlobalPosition>,
      public InterpolationMethod<VehicleGlobalPosition>
{
public:
    std::optional<VehicleGlobalPosition> convert(const px4_msgs::msg::VehicleGlobalPosition& message) const override
    {
        VehicleGlobalPosition result;
        result.timestamp = static_cast<int64_t>(message.timestamp_sample) * 1000;

        if (message.lat_lon_valid)
        {
            result.latitude = message.lat;
            result.longitude = message.lon;
        }

        if (message.alt_valid)
        {
            result.altitude_amsl = message.alt;
        }

        return result;
    }

    VehicleGlobalPosition interpolate(const VehicleGlobalPosition& a, const VehicleGlobalPosition& b, float alpha) const override
    {
        const double t = static_cast<double>(alpha);
        return {
            a.timestamp,
            a.latitude + t * (b.latitude - a.latitude),
            a.longitude + t * (b.longitude - a.longitude),
            a.altitude_amsl + alpha * (b.altitude_amsl - a.altitude_amsl)};
    }
};

}
