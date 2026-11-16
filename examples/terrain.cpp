#include <ifc_tools/processing/terrain_grid.hpp>
#include <ifc_tools/receiver/terrain_receiver.hpp>

#include <opencv2/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <memory>

namespace
{

cv::Mat draw(const ifc_tools::TerrainGrid& grid)
{
    cv::Mat z;
    cv::eigen2cv(grid.matrix(), z);
    cv::Mat elevation = grid.origin().ref_alt() - z;
    cv::flip(elevation, elevation, 0);

    const cv::Mat valid = elevation == elevation;
    double min = 0.0;
    double max = 0.0;
    cv::minMaxLoc(elevation, &min, &max, nullptr, nullptr, valid);

    cv::Mat normalized;
    elevation.convertTo(normalized, CV_8U, 255.0 / std::max(max - min, 1e-6), -255.0 * min / std::max(max - min, 1e-6));

    cv::Mat image;
    cv::applyColorMap(normalized, image, cv::COLORMAP_TURBO);
    image.setTo(cv::Scalar::all(0), ~valid);

    const double scale = 800.0 / std::max(image.rows, image.cols);
    cv::resize(image, image, cv::Size(), scale, scale, cv::INTER_NEAREST);

    char caption[160];
    std::snprintf(caption, sizeof(caption), "x %.0f:%.0f:%.0f y %.0f:%.0f:%.0f  %.1f-%.1f m AMSL",
        grid.x().start(), grid.x().step(), grid.x().stop(), grid.y().start(), grid.y().step(), grid.y().stop(), min, max);
    cv::putText(image, caption, cv::Point(10, 25), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar::all(255), 2);
    return image;
}

}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    auto receiver = std::make_shared<ifc_tools::TerrainReceiver>("terrain_viewer", "/terrain_model/terrain");
    std::shared_ptr<const ifc_tools::TerrainGrid> shown;

    auto timer = receiver->create_wall_timer(
        std::chrono::milliseconds(100),
        [&receiver, &shown]()
        {
            const std::shared_ptr<const ifc_tools::TerrainGrid> grid = receiver->grid();

            if (grid && grid != shown)
            {
                shown = grid;
                cv::imshow("terrain", draw(*grid));
            }

            cv::pollKey();
        });

    rclcpp::spin(receiver);
    rclcpp::shutdown();

    return 0;
}
