#pragma once

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstddef>
#include <cstdio>
#include <vector>
#include <stdexcept>

namespace ifc_tools
{

class DebugViewer
{
public:
    DebugViewer(cv::Size size, size_t rows, size_t cols)
        : _rows(rows),
          _cols(cols)
    {
        if (_rows == 0 || _cols == 0 ||
            size.width < 0 || size.height < 0 ||
            static_cast<size_t>(size.width) < _cols || static_cast<size_t>(size.height) < _rows)
        {
            throw std::invalid_argument("DebugViewer requires positive grid sizes and at least one pixel per cell");
        }

        _mosaic = cv::Mat::zeros(size, CV_8UC3);
        _cell_size = cv::Size(
            size.width / static_cast<int>(_cols),
            size.height / static_cast<int>(_rows));
        _stats.resize(_rows * _cols);
    }

    void update(size_t row, size_t col, const cv::Mat& mat)
    {
        if (row >= _rows || col >= _cols)
        {
            throw std::out_of_range("DebugViewer row or column is outside the configured grid");
        }

        cv::Mat image;

        if (!convert_color(mat, image))
        {
            throw std::invalid_argument("DebugViewer requires 8-bit gray, BGR, or BGRA images");
        }

        cv::Mat cell = _mosaic(cell_rect(row, col));
        fit(image, cell);
        ++_stats[row * _cols + col].frames;
    }

    void render(uint64_t timestamp_us)
    {
        calculate_fps(timestamp_us);

        cv::Mat frame = _mosaic.clone();
        draw_fps(frame);

        cv::imshow("DebugViewer", frame);
        cv::pollKey();
    }

private:
    static constexpr double fps_text_height_fraction = 0.05;
    static constexpr uint64_t fps_window_us = 1000000;

    struct Stats
    {
        uint64_t frames = 0;
        uint64_t window_start_us = 0;
        double fps = 0.0;
        bool started = false;
    };

    // The last row/column absorbs the remainder pixels.
    cv::Rect cell_rect(size_t row, size_t col) const
    {
        const int left = static_cast<int>(col) * _cell_size.width;
        const int top = static_cast<int>(row) * _cell_size.height;
        const int width = col == _cols - 1 ? _mosaic.cols - left : _cell_size.width;
        const int height = row == _rows - 1 ? _mosaic.rows - top : _cell_size.height;
        return cv::Rect(left, top, width, height);
    }

    void calculate_fps(uint64_t timestamp_us)
    {
        for (Stats& stats : _stats)
        {
            if (!stats.started)
            {
                stats.started = true;
                stats.window_start_us = timestamp_us;
                continue;
            }

            const uint64_t elapsed_us = timestamp_us - stats.window_start_us;

            if (elapsed_us < fps_window_us)
            {
                continue;
            }

            stats.fps = stats.frames * 1e6 / static_cast<double>(elapsed_us);
            stats.frames = 0;
            stats.window_start_us = timestamp_us;
        }
    }

    void draw_fps(cv::Mat& frame) const
    {
        for (size_t row = 0; row < _rows; ++row)
        {
            for (size_t col = 0; col < _cols; ++col)
            {
                cv::Mat cell = frame(cell_rect(row, col));

                const double font_scale = std::max(0.3, cell.rows * fps_text_height_fraction / 22.0);
                const int thickness = std::max(1, static_cast<int>(std::round(font_scale * 1.5)));
                char text[32];
                std::snprintf(text, sizeof(text), "%.1f FPS", _stats[row * _cols + col].fps);
                int baseline = 0;
                const cv::Size text_size = cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX, font_scale, thickness, &baseline);
                const int margin = std::max(2, cell.rows / 100);
                const cv::Point origin(cell.cols - text_size.width - margin, margin + text_size.height);
                cv::putText(
                    cell, text, origin, cv::FONT_HERSHEY_SIMPLEX, font_scale,
                    cv::Scalar(0, 255, 0), thickness, cv::LINE_AA);
            }
        }
    }

    static bool convert_color(const cv::Mat& mat, cv::Mat& out)
    {
        if (mat.empty())
        {
            out.release();
            return true;
        }

        if (mat.depth() != CV_8U ||
            (mat.channels() != 1 && mat.channels() != 3 && mat.channels() != 4))
        {
            return false;
        }

        if (mat.channels() == 1)
        {
            cv::cvtColor(mat, out, cv::COLOR_GRAY2BGR);
        }
        else if (mat.channels() == 4)
        {
            cv::cvtColor(mat, out, cv::COLOR_BGRA2BGR);
        }
        else
        {
            out = mat;
        }

        return true;
    }

    static void fit(const cv::Mat& image, cv::Mat& cell)
    {
        cell.setTo(cv::Scalar::all(0));

        if (image.empty())
        {
            return;
        }

        const double scale = std::min(
            static_cast<double>(cell.cols) / image.cols,
            static_cast<double>(cell.rows) / image.rows);
        const cv::Size scaled_size(
            std::clamp(static_cast<int>(std::round(image.cols * scale)), 1, cell.cols),
            std::clamp(static_cast<int>(std::round(image.rows * scale)), 1, cell.rows));
        cv::Mat resized;
        cv::resize(
            image, resized, scaled_size, 0.0, 0.0,
            scale < 1.0 ? cv::INTER_AREA : cv::INTER_LINEAR);
        const cv::Rect region(
            (cell.cols - scaled_size.width) / 2,
            (cell.rows - scaled_size.height) / 2,
            scaled_size.width, scaled_size.height);
        resized.copyTo(cell(region));
    }

    size_t _rows;
    size_t _cols;
    cv::Mat _mosaic;
    cv::Size _cell_size;
    std::vector<Stats> _stats;
};

}
