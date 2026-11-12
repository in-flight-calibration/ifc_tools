#pragma once

#include <ifc_tools/processing/interpolator.hpp>
#include <ifc_tools/receiver/buffered_receiver.hpp>

#include <px4_msgs/msg/vehicle_angular_velocity.hpp>

#include <Eigen/Core>

#include <cstdint>
#include <optional>

namespace ifc_tools
{

struct VehicleAngularVelocity
{
    int64_t timestamp = 0;
    Eigen::Vector3d omega = Eigen::Vector3d::Zero();
};

using VehicleAngularVelocityReceiver = BufferedReceiver<px4_msgs::msg::VehicleAngularVelocity, VehicleAngularVelocity>;

class VehicleAngularVelocityUtils
    : public MessageConverter<px4_msgs::msg::VehicleAngularVelocity, VehicleAngularVelocity>,
      public InterpolationMethod<VehicleAngularVelocity>
{
public:
    std::optional<VehicleAngularVelocity> convert(const px4_msgs::msg::VehicleAngularVelocity& message) const override
    {
        return VehicleAngularVelocity{
            static_cast<int64_t>(message.timestamp_sample) * 1000,
            Eigen::Vector3d(message.xyz[0], message.xyz[1], message.xyz[2])};
    }

    VehicleAngularVelocity interpolate(const VehicleAngularVelocity& a, const VehicleAngularVelocity& b, float alpha) const override
    {
        return {a.timestamp, a.omega + static_cast<double>(alpha) * (b.omega - a.omega)};
    }
};

}
