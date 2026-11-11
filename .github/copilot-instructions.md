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

The tests are the headless CTests `ifc_tools_ring_buffer` and `ifc_tools_interpolator` (`colcon test --packages-select ifc_tools --ctest-args -R interpolator` runs one); there are no linters. Verify GUI/ROS code by building and running the example. The example needs a FlightGear publisher on `/flightgear/image` (`bgr8`, `SensorDataQoS`), the `compressed_image_transport` plugin, and a desktop. Warnings are enabled (`-Wall -Wextra -Wpedantic`); keep the build warning-free.

## Architecture
- Single `INTERFACE` CMake target `ifc_tools` (exported via `ament_export_targets`); all code is in headers under `include/ifc_tools/`, namespace `ifc_tools`. New dependencies must be added to `CMakeLists.txt` (`find_package`, link, `ament_export_dependencies`) and `package.xml`.
- `examples/*.cpp` are built as executables and installed to `lib/ifc_tools`; register new ones in `CMakeLists.txt`.
- `ImageReceiver` (an `rclcpp::Node`): subscribes via `image_transport::create_subscription(*this, topic, cb, transport, rclcpp::SensorDataQoS())` (the `rclcpp::Node*` overload is deprecated). Converts with `cv_bridge::toCvShare(msg, "bgr8")` and calls a mutex-protected `std::function<void(const rclcpp::Time&, const cv::Mat&)>` set with `setCallback`. The `cv::Mat` is only valid during the callback; copy it to keep it.
- `DebugViewer(cv::Size, size_t rows, size_t cols)`: fixed-grid mosaic in one OpenCV window. `update(row, col, mat)` (zero-based) copies/letterboxes an image into a cell and counts a frame for that tile; `render(timestamp_us)` takes a monotonic `uint64_t` time in microseconds, recomputes each tile's FPS once per second, draws a green FPS HUD on a clone of the mosaic (the stored mosaic stays clean), then `imshow` + `pollKey`. Accepts 8-bit gray/BGR/BGRA only. Single-thread use (the callback thread).
- The horizon detector base currently stores an optional `DebugViewer`; its construction and use are not wired yet.
- `RingBuffer<T>(capacity)`: fixed-capacity FIFO that overwrites the oldest element when full. Storage for `capacity` elements is reserved in the constructor, so `push` never reallocates. Index 0 is the oldest; provides `operator[]`, a const forward iterator (`begin`/`end`) and `toVector`. Not thread-safe; callers lock. Tested by CTest `ifc_tools_ring_buffer` (`colcon build --packages-select ifc_tools --cmake-args -DBUILD_TESTING=ON && colcon test --packages-select ifc_tools`).
- `interpolator.hpp`: `InterpolationMethod<T>` (pure virtual `interpolate(a, b, float alpha) -> T`, a classic abstract interface, not traits) with the default `Nearest<T>`; and `Interpolator<Container>`, a non-owning decorator (holds a const reference; binding a temporary is deleted) of any iterable container of elements `T` with a public `timestamp` member, sorted by ascending timestamp (the timestamp type is deduced from `T::timestamp`). The method interpolates whole elements. `interpolate(timestamp, method = Nearest<T>())` and `interpolate(ascending_timestamps, method = Nearest<T>())` return `std::optional<T>` (per query for the vector) using a forward-only bracket, with `timestamp` set to the query; outside the time range (before the first or after the last sample) they return `std::nullopt`. An empty container throws `std::out_of_range`. Use with CTAD: `ifc_tools::Interpolator(samples).interpolate(ts, method)`.
- `MessageConverter<Message, Value>` (`buffered_receiver.hpp`; pure virtual `convert(message) -> std::optional<Value>`, where `Value` has a public `timestamp` member; empty rejects the message).
- `BufferedReceiver<Message, Value>(node_name, converter, capacity, topic, qos = SensorDataQoS)`: an `rclcpp::Node` (like `ImageReceiver`; add it to the executor) that converts each message and pushes it into a mutex-protected `RingBuffer<Value>`. It drops a sample whose timestamp equals the last one and clears the buffer (with a warning) when the timestamp goes backwards (simulation reset). `snapshot()` returns the buffer as a `std::vector<Value>` for computation off the callback; decorate it with `Interpolator`.

## Conventions
- Brace-on-own-line (Allman) style, 4-space indent, `_`-prefixed private members (`_callback_mutex`), `#pragma once`.
