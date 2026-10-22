#include <ifc_tools/debug_viewer.hpp>
#include <ifc_tools/image_receiver.hpp>

#include <chrono>
#include <cstdint>
#include <memory>

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    ifc_tools::DebugViewer viewer(cv::Size(1280, 720), 2, 2);
    auto receiver = std::make_shared<ifc_tools::ImageReceiver>(
        "image_viewer", "/flightgear/image", "compressed");

    const auto now_us = []() -> uint64_t
    {
        return std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
    };

    receiver->setCallback(
        [&viewer, now_us](const rclcpp::Time&, const cv::Mat& image)
        {
            for (size_t row = 0; row < 2; ++row)
            {
                for (size_t col = 0; col < 2; ++col)
                {
                    viewer.update(row, col, image);
                }
            }

            viewer.render(now_us());
        });

    auto timer = receiver->create_wall_timer(
        std::chrono::seconds(1),
        [&viewer, now_us]()
        {
            viewer.render(now_us());
        });

    rclcpp::spin(receiver);
    rclcpp::shutdown();

    return 0;
}
