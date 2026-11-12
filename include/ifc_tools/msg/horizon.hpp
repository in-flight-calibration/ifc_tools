#pragma once

#include <ifc_tools/receiver/buffered_receiver.hpp>

#include <ifc_msgs/msg/horizon.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/time.hpp>

#include <array>
#include <cmath>
#include <cstdint>
#include <optional>
#include <vector>

namespace ifc_tools
{

struct Horizon
{
    int64_t timestamp = 0;
    float theta = 0.0f;
    float rho = 0.0f;
};

using HorizonReceiver = BufferedReceiver<ifc_msgs::msg::Horizon, Horizon>;

class HorizonUtils
    : public MessageConverter<ifc_msgs::msg::Horizon, Horizon>
{
public:
    std::optional<Horizon> convert(const ifc_msgs::msg::Horizon& message) const override
    {
        if (message.confidence <= 0.0f || !std::isfinite(message.theta) || !std::isfinite(message.rho))
        {
            return std::nullopt;
        }

        return Horizon{
            rclcpp::Time(message.image_timestamp).nanoseconds(),
            message.theta,
            message.rho};
    }

    static cv::Mat groundMask(cv::Size size, float theta, float rho, float margin_px)
    {
        const double half_diagonal = 0.5 * std::hypot(size.width, size.height);
        const double offset = rho - margin_px / half_diagonal;
        const std::array<cv::Point2d, 4> corners = {
            cv::Point2d(0, 0), cv::Point2d(size.width, 0),
            cv::Point2d(size.width, size.height), cv::Point2d(0, size.height)};

        const auto signedDistance = [&](const cv::Point2d& p)
        {
            const double u = (p.x - 0.5 * size.width) / half_diagonal;
            const double v = (0.5 * size.height - p.y) / half_diagonal;
            return -u * std::sin(theta) + v * std::cos(theta) - offset;
        };

        std::vector<cv::Point> polygon;

        for (size_t i = 0; i < corners.size(); ++i)
        {
            const cv::Point2d& a = corners[i];
            const cv::Point2d& b = corners[(i + 1) % corners.size()];
            const double da = signedDistance(a);
            const double db = signedDistance(b);

            if (da < 0.0)
            {
                polygon.emplace_back(cvRound(a.x), cvRound(a.y));
            }

            if ((da < 0.0) != (db < 0.0))
            {
                const cv::Point2d p = a + (b - a) * (da / (da - db));
                polygon.emplace_back(cvRound(p.x), cvRound(p.y));
            }
        }

        cv::Mat mask = cv::Mat::zeros(size, CV_8UC1);

        if (polygon.size() >= 3)
        {
            cv::fillConvexPoly(mask, polygon, cv::Scalar(255));
        }

        return mask;
    }
};

}
