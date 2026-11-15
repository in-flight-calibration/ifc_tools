#pragma once

#include <ifc_tools/processing/interpolator.hpp>
#include <ifc_tools/receiver/buffered_receiver.hpp>

#include <px4_msgs/msg/vehicle_global_position.hpp>

#include <Eigen/Dense>

#include <cmath>
#include <cstdint>
#include <limits>
#include <numbers>
#include <optional>
#include <utility>

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
        constexpr double to_rad = std::numbers::pi / 180.0;

        const auto to_vector = [](double latitude, double longitude)
        {
            const double lat = latitude * to_rad;
            const double lon = longitude * to_rad;
            return Eigen::Vector3d(std::cos(lat) * std::cos(lon), std::cos(lat) * std::sin(lon), std::sin(lat));
        };

        const auto to_lat_lon = [](const Eigen::Vector3d& n)
        {
            return std::pair<double, double> {std::atan2(n.z(), std::hypot(n.x(), n.y())) / to_rad, std::atan2(n.y(), n.x()) / to_rad};
        };

        const Eigen::Vector3d n = ((1.0 - t) * to_vector(a.latitude, a.longitude) + t * to_vector(b.latitude, b.longitude)).normalized();
        const auto [latitude, longitude] = to_lat_lon(n);
        return {
            a.timestamp,
            latitude,
            longitude,
            a.altitude_amsl + alpha * (b.altitude_amsl - a.altitude_amsl)};
    }
};

}
