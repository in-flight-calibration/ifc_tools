#pragma once

#include <ifc_tools/processing/interpolator.hpp>
#include <ifc_tools/receiver/buffered_receiver.hpp>

#include <px4_msgs/msg/vehicle_attitude.hpp>

#include <Eigen/Geometry>

#include <cmath>
#include <cstdint>
#include <optional>

namespace ifc_tools
{

struct VehicleAttitude
{
    int64_t timestamp = 0;
    Eigen::Quaterniond q = Eigen::Quaterniond::Identity();
};

using VehicleAttitudeReceiver = BufferedReceiver<px4_msgs::msg::VehicleAttitude, VehicleAttitude>;

class VehicleAttitudeUtils
    : public MessageConverter<px4_msgs::msg::VehicleAttitude, VehicleAttitude>,
      public InterpolationMethod<VehicleAttitude>
{
public:
    std::optional<VehicleAttitude> convert(const px4_msgs::msg::VehicleAttitude& message) const override
    {
        const Eigen::Quaterniond q(message.q[0], message.q[1], message.q[2], message.q[3]);
        const double norm = q.norm();

        if (!std::isfinite(norm) || norm <= 0.0)
        {
            return std::nullopt;
        }

        return VehicleAttitude{static_cast<int64_t>(message.timestamp_sample) * 1000, q.normalized()};
    }

    VehicleAttitude interpolate(const VehicleAttitude& a, const VehicleAttitude& b, float alpha) const override
    {
        Eigen::Quaterniond delta = a.q.conjugate() * b.q;

        if (delta.w() < 0.0)
        {
            delta.coeffs() = -delta.coeffs();
        }

        const Eigen::AngleAxisd rotation(delta);
        return {a.timestamp, (a.q * Eigen::Quaterniond(Eigen::AngleAxisd(alpha * rotation.angle(), rotation.axis()))).normalized()};
    }
};

}
