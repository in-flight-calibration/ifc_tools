#pragma once

#include <rclcpp/time.hpp>

#include <cstdint>
#include <limits>
#include <optional>

namespace ifc_tools
{


class PerformanceMonitor
{
public:
    struct Statistics
    {
        uint64_t count{0};
        int64_t min_ns{0};
        int64_t max_ns{INT64_MIN};
        int64_t avg_ns{0};
    };

    bool start(const rclcpp::Time& timestamp)
    {
        if (_start_time) {
            return false;
        }

        _start_time = timestamp;
        return true;
    }

    bool stop(const rclcpp::Time& timestamp)
    {
        if (!_start_time) {
            return false;
        }

        const int64_t duration_ns = (timestamp - *_start_time).nanoseconds();
        _start_time.reset();

        if (duration_ns < 0 ||
            _stats.count == std::numeric_limits<uint64_t>::max()) {
            return false;
        }

        _stats.count++;
        _stats.min_ns = std::min(_stats.min_ns, duration_ns);
        _stats.max_ns = std::max(_stats.max_ns, duration_ns);
        _stats.avg_ns = (_stats.avg_ns * (_stats.count - 1) + duration_ns) / static_cast<int64_t>(_stats.count);
        return true;
    }

    void reset()
    {
        _start_time.reset();
        _stats = {};
    }

    const Statistics& statistics() const
    {
        return _stats;
    }

    bool running() const
    {
        return _start_time.has_value();
    }

private:
    std::optional<rclcpp::Time> _start_time;
    Statistics _stats{};
};

}
