#pragma once

#include <Eigen/Core>

#include <algorithm>
#include <cfloat>
#include <cmath>
#include <cstddef>
#include <initializer_list>
#include <limits>
#include <optional>
#include <stdexcept>
#include <utility>
#include <vector>

namespace ifc_tools
{

class TerrainOrigin
{
public:
    TerrainOrigin(double ref_lat, double ref_lon, float ref_alt)
        : _ref_lat(ref_lat), _ref_lon(ref_lon), _ref_alt(ref_alt)
    {
        if (!(std::abs(ref_lat) <= 90.0) || !(std::abs(ref_lon) <= 180.0) || !std::isfinite(ref_alt))
        {
            throw std::invalid_argument("TerrainOrigin: latitude must be in [-90, 90], longitude in [-180, 180] and altitude finite");
        }
    }

    double ref_lat() const
    {
        return _ref_lat;
    }

    double ref_lon() const
    {
        return _ref_lon;
    }

    float ref_alt() const
    {
        return _ref_alt;
    }

    bool operator==(const TerrainOrigin&) const = default;

private:
    double _ref_lat;
    double _ref_lon;
    float _ref_alt;
};

class TerrainAxis
{
public:
    TerrainAxis(float start, float step, float stop)
        : _start(start), _step(step), _stop(stop)
    {
        if (!std::isfinite(start) || !std::isfinite(stop) || !(step > FLT_EPSILON) || step > (stop - start + FLT_EPSILON))
        {
            throw std::invalid_argument("TerrainAxis: start:step:stop must be finite with step > 0 and at least 2 values");
        }

        _size = static_cast<std::size_t>(std::floor((stop - start) / step)) + 1;
    }

    float start() const
    {
        return _start;
    }

    float step() const
    {
        return _step;
    }

    float stop() const
    {
        return _stop;
    }

    std::size_t size() const
    {
        return _size;
    }

private:
    float _start;
    float _step;
    float _stop;
    std::size_t _size;
};

class TerrainGrid
{
public:
    TerrainGrid(const TerrainOrigin& origin, const TerrainAxis& x, const TerrainAxis& y, const std::vector<float>& z)
        : TerrainGrid(origin, x, y, toMatrix(x, y, z))
    {
    }

    TerrainGrid(const TerrainOrigin& origin, const TerrainAxis& x, const TerrainAxis& y, Eigen::MatrixXd z)
        : _origin(origin), _x(x), _y(y), _z(std::move(z))
    {
        if (_z.rows() != static_cast<Eigen::Index>(_x.size()) || _z.cols() != static_cast<Eigen::Index>(_y.size()))
        {
            throw std::invalid_argument("TerrainGrid: z must have x.size() rows and y.size() columns");
        }

        for (const double value : _z.reshaped())
        {
            if (!std::isnan(value))
            {
                _z_min = std::min(_z_min, value);
                _z_max = std::max(_z_max, value);
            }
        }
    }

    const TerrainAxis& x() const
    {
        return _x;
    }

    const TerrainAxis& y() const
    {
        return _y;
    }

    const TerrainOrigin& origin() const
    {
        return _origin;
    }

    const Eigen::MatrixXd& matrix() const
    {
        return _z;
    }

    std::vector<float> toVector() const
    {
        std::vector<float> z(static_cast<std::size_t>(_z.size()));
        Eigen::Map<Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(z.data(), _z.rows(), _z.cols()) = _z.cast<float>();
        return z;
    }

    double z(double x, double y) const
    {
        const double u = (x - _x.start()) / _x.step();
        const double v = (y - _y.start()) / _y.step();
        const double max_u = static_cast<double>(_x.size() - 1);
        const double max_v = static_cast<double>(_y.size() - 1);

        if (!(u >= 0.0 && u <= max_u && v >= 0.0 && v <= max_v))
        {
            return std::numeric_limits<double>::quiet_NaN();
        }

        const Eigen::Index i = cell(u, _x.size());
        const Eigen::Index j = cell(v, _y.size());
        const double a = u - static_cast<double>(i);
        const double b = v - static_cast<double>(j);

        const double z00 = _z(i, j);
        const double z01 = _z(i, j + 1);
        const double z10 = _z(i + 1, j);
        const double z11 = _z(i + 1, j + 1);

        return (1.0 - a) * ((1.0 - b) * z00 + b * z01) + a * ((1.0 - b) * z10 + b * z11);
    }

    std::optional<Eigen::Vector3d> raycast(const Eigen::Vector3d& p, const Eigen::Vector3d& v) const
    {
        if (!p.allFinite() || !v.allFinite() || v.isZero())
        {
            return std::nullopt;
        }

        const double u0 = (p.x() - _x.start()) / _x.step();
        const double w0 = (p.y() - _y.start()) / _y.step();
        const double du = v.x() / _x.step();
        const double dw = v.y() / _y.step();

        double t_enter = 0.0;
        double t_exit = std::numeric_limits<double>::infinity();

        if (!clip(u0, du, static_cast<double>(_x.size() - 1), t_enter, t_exit)
            || !clip(w0, dw, static_cast<double>(_y.size() - 1), t_enter, t_exit))
        {
            return std::nullopt;
        }

        if (v.z() > 0.0)
        {
            t_exit = std::min(t_exit, (_z_max - p.z()) / v.z());
        }
        else if (p.z() < _z_min)
        {
            return std::nullopt;
        }

        if (!(t_enter <= t_exit))
        {
            return std::nullopt;
        }

        const int su = du > 0.0 ? 1 : -1;
        const int sw = dw > 0.0 ? 1 : -1;
        const double t_delta_u = du != 0.0 ? 1.0 / std::abs(du) : std::numeric_limits<double>::infinity();
        const double t_delta_w = dw != 0.0 ? 1.0 / std::abs(dw) : std::numeric_limits<double>::infinity();

        Eigen::Index i = cell(u0 + du * t_enter, _x.size());
        Eigen::Index j = cell(w0 + dw * t_enter, _y.size());
        double t_next_u = du != 0.0 ? (static_cast<double>(i + (su > 0 ? 1 : 0)) - u0) / du : std::numeric_limits<double>::infinity();
        double t_next_w = dw != 0.0 ? (static_cast<double>(j + (sw > 0 ? 1 : 0)) - w0) / dw : std::numeric_limits<double>::infinity();
        double t = t_enter;

        while (true)
        {
            const double t_cell_exit = std::min({t_next_u, t_next_w, t_exit});
            const std::optional<double> hit = intersect(i, j, u0 + du * t, w0 + dw * t, du, dw, p.z() + v.z() * t, v.z(), t_cell_exit - t);

            if (!hit || *hit < 0.0)
            {
                return std::nullopt;
            }

            if (*hit <= t_cell_exit - t)
            {
                if (*hit == 0.0 && t == t_enter)
                {
                    return std::nullopt;
                }

                return p + (t + *hit) * v;
            }

            if (t_cell_exit >= t_exit)
            {
                return std::nullopt;
            }

            if (t_next_u < t_next_w)
            {
                i += su;
                t = t_next_u;
                t_next_u += t_delta_u;
            }
            else
            {
                j += sw;
                t = t_next_w;
                t_next_w += t_delta_w;
            }

            if (i < 0 || j < 0 || i > static_cast<Eigen::Index>(_x.size()) - 2 || j > static_cast<Eigen::Index>(_y.size()) - 2)
            {
                return std::nullopt;
            }
        }
    }

private:
    TerrainOrigin _origin;
    TerrainAxis _x;
    TerrainAxis _y;
    Eigen::MatrixXd _z;
    double _z_min = std::numeric_limits<double>::infinity();
    double _z_max = -std::numeric_limits<double>::infinity();

    std::optional<double> intersect(
        Eigen::Index i, Eigen::Index j, double u, double w, double du, double dw, double ray_z, double dz, double length) const
    {
        const double z00 = _z(i, j);
        const double z01 = _z(i, j + 1);
        const double z10 = _z(i + 1, j);
        const double z11 = _z(i + 1, j + 1);

        if (std::isnan(z00) || std::isnan(z01) || std::isnan(z10) || std::isnan(z11))
        {
            return std::nullopt;
        }

        const double a0 = u - static_cast<double>(i);
        const double b0 = w - static_cast<double>(j);
        const double ca = z10 - z00;
        const double cb = z01 - z00;
        const double cab = z00 - z10 - z01 + z11;

        const double A = cab * du * dw;
        const double B = ca * du + cb * dw + cab * (a0 * dw + b0 * du) - dz;
        const double C = z00 + ca * a0 + cb * b0 + cab * a0 * b0 - ray_z;

        if (!(C > 0.0))
        {
            return 0.0;
        }

        const double discriminant = B * B - 4.0 * A * C;

        if (discriminant < 0.0)
        {
            return std::numeric_limits<double>::infinity();
        }

        const double q = -0.5 * (B + std::copysign(std::sqrt(discriminant), B));
        double first = std::numeric_limits<double>::infinity();

        for (const double root : {q / A, C / q})
        {
            if (std::isfinite(root) && root >= 0.0 && root <= length)
            {
                first = std::min(first, root);
            }
        }

        return first;
    }

    static bool clip(double origin, double direction, double max, double& t_enter, double& t_exit)
    {
        if (direction == 0.0)
        {
            return origin >= 0.0 && origin <= max;
        }

        const double t_min = (0.0 - origin) / direction;
        const double t_max = (max - origin) / direction;
        t_enter = std::max(t_enter, std::min(t_min, t_max));
        t_exit = std::min(t_exit, std::max(t_min, t_max));
        return t_enter <= t_exit;
    }

    static Eigen::Index cell(double index, std::size_t size)
    {
        return std::clamp(static_cast<Eigen::Index>(std::floor(index)), Eigen::Index {0}, static_cast<Eigen::Index>(size) - 2);
    }

    static Eigen::MatrixXd toMatrix(const TerrainAxis& x, const TerrainAxis& y, const std::vector<float>& z)
    {
        if (z.size() != x.size() * y.size())
        {
            throw std::invalid_argument("TerrainGrid: z must have x.size() * y.size() values");
        }

        return Eigen::Map<const Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
            z.data(), static_cast<Eigen::Index>(x.size()), static_cast<Eigen::Index>(y.size())).cast<double>();
    }
};

}
