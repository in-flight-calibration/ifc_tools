#pragma once

#include <ifc_tools/processing/terrain_grid.hpp>

#include <ifc_msgs/msg/terrain.hpp>
#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>

namespace ifc_tools
{

class TerrainReceiver : public rclcpp::Node
{
public:
    TerrainReceiver(const std::string& node_name, const std::string& topic)
        : rclcpp::Node(node_name)
    {
        _subscription = create_subscription<ifc_msgs::msg::Terrain>(
            topic, rclcpp::QoS(1).transient_local().reliable(),
            [this](const ifc_msgs::msg::Terrain::ConstSharedPtr message)
            {
                receive(*message);
            });
    }

    std::shared_ptr<const TerrainGrid> grid() const
    {
        std::scoped_lock<std::mutex> lock(_grid_mtx);
        return _grid;
    }

private:
    mutable std::mutex _grid_mtx;
    std::shared_ptr<const TerrainGrid> _grid;

    rclcpp::Subscription<ifc_msgs::msg::Terrain>::SharedPtr _subscription;

    void receive(const ifc_msgs::msg::Terrain& message)
    {
        try
        {
            auto grid = std::make_shared<const TerrainGrid>(
                TerrainOrigin(message.ref_lat, message.ref_lon, message.ref_alt),
                TerrainAxis(message.x[0], message.x[1], message.x[2]),
                TerrainAxis(message.y[0], message.y[1], message.y[2]),
                message.z);

            std::scoped_lock<std::mutex> lock(_grid_mtx);
            _grid = std::move(grid);
        }
        catch (const std::invalid_argument& error)
        {
            RCLCPP_WARN(get_logger(), "Terrain rejected: %s", error.what());
        }
    }
};

}
