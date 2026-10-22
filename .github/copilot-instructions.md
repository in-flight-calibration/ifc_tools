# Copilot instructions: ifc_tools

Header-only ROS 2 (lyrical) + OpenCV 4.10 utility package, C++20, for the `ifc` colcon workspace (this repo lives at `<ws>/src/ifc_tools`). First consumer is `horizon_detection`.

## Build / run
Run from the workspace root (two levels up), not this directory:

```bash
source /opt/ros/lyrical/setup.bash
colcon build --packages-select ifc_tools
source install/setup.bash
ros2 run ifc_tools mosaic
```

`mosaic` also calls `render` from a 1 s wall timer so the window and FPS refresh when no frames arrive.

There are no tests or linters (GUI/ROS code); verify by building and running the example. The example needs a FlightGear publisher on `/flightgear/image` (`bgr8`, `SensorDataQoS`), the `compressed_image_transport` plugin, and a desktop. Warnings are enabled (`-Wall -Wextra -Wpedantic`); keep the build warning-free.

## Architecture
- Single `INTERFACE` CMake target `ifc_tools` (exported via `ament_export_targets`); all code is in headers under `include/ifc_tools/`, namespace `ifc_tools`. New dependencies must be added to `CMakeLists.txt` (`find_package`, link, `ament_export_dependencies`) and `package.xml`.
- `examples/*.cpp` are built as executables and installed to `lib/ifc_tools`; register new ones in `CMakeLists.txt`.
- `ImageReceiver` (an `rclcpp::Node`): subscribes via `image_transport::create_subscription(*this, topic, cb, transport, rclcpp::SensorDataQoS())` (the `rclcpp::Node*` overload is deprecated). Converts with `cv_bridge::toCvShare(msg, "bgr8")` and calls a mutex-protected `std::function<void(const rclcpp::Time&, const cv::Mat&)>` set with `setCallback`. The `cv::Mat` is only valid during the callback; copy it to keep it.
- `DebugViewer(cv::Size, size_t rows, size_t cols)`: fixed-grid mosaic in one OpenCV window. `update(row, col, mat)` (zero-based) copies/letterboxes an image into a cell and counts a frame for that tile; `render(timestamp_us)` takes a monotonic `uint64_t` time in microseconds, recomputes each tile's FPS once per second, draws a green FPS HUD on a clone of the mosaic (the stored mosaic stays clean), then `imshow` + `pollKey`. Accepts 8-bit gray/BGR/BGRA only. Single-thread use (the callback thread).
- Consumers take the viewer as `std::optional<std::reference_wrapper<DebugViewer>>` (no `optional<T&>` in C++20); `std::nullopt` disables debugging, so drawing helpers must return immediately in that case.

## Conventions
- Brace-on-own-line (Allman) style, 4-space indent, `_`-prefixed private members (`_callback_mutex`), `#pragma once`.
