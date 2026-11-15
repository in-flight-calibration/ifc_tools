#include <ifc_tools/processing/terrain_grid.hpp>

#include <Eigen/Core>

#include <cmath>
#include <cstddef>
#include <cstdio>
#include <optional>
#include <stdexcept>
#include <vector>

namespace
{

bool check(bool condition, const char* name)
{
    std::printf("%s: %s\n", condition ? "PASS" : "FAIL", name);
    return condition;
}

template <typename Function>
bool throws(Function function)
{
    try
    {
        function();
    }
    catch (const std::invalid_argument&)
    {
        return true;
    }

    return false;
}

bool near(double a, double b, double tolerance = 1e-6)
{
    return std::abs(a - b) <= tolerance;
}

ifc_tools::TerrainGrid slope()
{
    const ifc_tools::TerrainAxis x(-100.0f, 10.0f, 100.0f);
    const ifc_tools::TerrainAxis y(-50.0f, 10.0f, 50.0f);
    std::vector<float> z(x.size() * y.size());

    for (std::size_t i = 0; i < x.size(); ++i)
    {
        for (std::size_t j = 0; j < y.size(); ++j)
        {
            z[i * y.size() + j] = 10.0f - 0.1f * (x.start() + x.step() * i) + 0.2f * (y.start() + y.step() * j);
        }
    }

    return ifc_tools::TerrainGrid(ifc_tools::TerrainOrigin(52.0, 21.0, 100.0f), x, y, z);
}

}

int main()
{
    bool ok = true;
    const ifc_tools::TerrainGrid grid = slope();

    ok &= check(grid.x().size() == 21 && grid.y().size() == 11, "axis size from start:step:stop");
    ok &= check(grid.origin().ref_lat() == 52.0 && grid.origin().ref_alt() == 100.0f, "keeps the origin");
    ok &= check(near(grid.z(0.0, 0.0), 10.0) && near(grid.z(-100.0, -50.0), 10.0 + 10.0 - 10.0), "z at grid nodes");
    ok &= check(near(grid.z(13.0, -7.0), 10.0 - 1.3 - 1.4, 1e-5), "bilinear z between nodes");
    ok &= check(near(grid.z(100.0, 50.0), 10.0 - 10.0 + 10.0, 1e-5), "z on the far edge");
    ok &= check(std::isnan(grid.z(100.1, 0.0)) && std::isnan(grid.z(0.0, -50.1)), "NaN outside the grid");

    const Eigen::Vector3d p(0.0, 0.0, -50.0);
    const std::optional<Eigen::Vector3d> down = grid.raycast(p, Eigen::Vector3d(0.0, 0.0, 2.0));
    ok &= check(down && near(down->z(), 10.0, 1e-6) && near(down->x(), 0.0), "vertical ray hits the terrain below");

    const Eigen::Vector3d v(1.0, 0.5, 1.0);
    const std::optional<Eigen::Vector3d> oblique = grid.raycast(p, v);
    ok &= check(oblique && near(oblique->z(), grid.z(oblique->x(), oblique->y()), 1e-6), "oblique hit lies on the terrain");
    ok &= check(oblique && near((*oblique - p).normalized().dot(v.normalized()), 1.0, 1e-9), "oblique hit lies on the ray");

    ok &= check(!grid.raycast(p, Eigen::Vector3d(0.0, 0.0, -1.0)), "upward ray misses");
    const Eigen::Vector3d outside(-150.0, 0.0, -10.0);
    const std::optional<Eigen::Vector3d> entering = grid.raycast(outside, Eigen::Vector3d(1.0, 0.0, 0.05));
    ok &= check(entering && entering->x() > -100.0 && near(entering->z(), grid.z(entering->x(), entering->y()), 1e-6), "ray from outside enters the grid and hits");

    std::vector<float> flat(21 * 21, 0.0f);
    flat[10 * 21 + 10] = -20.0f;
    const ifc_tools::TerrainAxis axis(-100.0f, 10.0f, 100.0f);
    const ifc_tools::TerrainGrid peak(ifc_tools::TerrainOrigin(0.0, 0.0, 0.0f), axis, axis, flat);
    const std::optional<Eigen::Vector3d> front = peak.raycast(Eigen::Vector3d(-100.0, 3.0, -10.0), Eigen::Vector3d(1.0, 0.0, 0.0));
    ok &= check(front && near(front->x(), -10.0 + 10.0 * 10.0 / 14.0, 1e-9), "horizontal ray hits the front of a single-node peak");
    ok &= check(!peak.raycast(Eigen::Vector3d(-100.0, 3.0, -15.0), Eigen::Vector3d(1.0, 0.0, 0.0)), "horizontal ray above the peak misses");
    ok &= check(!grid.raycast(p, Eigen::Vector3d(1.0, 0.0, 0.01)), "ray leaving the grid misses");
    ok &= check(!grid.raycast(Eigen::Vector3d(0.0, 0.0, 20.0), Eigen::Vector3d(0.0, 0.0, 1.0)), "start below the terrain misses");
    ok &= check(!grid.raycast(p, Eigen::Vector3d::Zero()), "zero direction misses");

    std::vector<float> holes(3 * 2, 0.0f);
    holes[2 * 2 + 1] = std::nanf("");
    const ifc_tools::TerrainGrid partial(ifc_tools::TerrainOrigin(0.0, 0.0, 0.0f), ifc_tools::TerrainAxis(0.0f, 10.0f, 20.0f), ifc_tools::TerrainAxis(0.0f, 10.0f, 10.0f), holes);
    ok &= check(near(partial.z(5.0, 5.0), 0.0) && std::isnan(partial.z(15.0, 5.0)), "NaN corner makes its cell NaN");
    ok &= check(!partial.raycast(Eigen::Vector3d(15.0, 5.0, -10.0), Eigen::Vector3d(0.0, 0.0, 1.0)), "ray into no data misses");

    ok &= check(ifc_tools::TerrainAxis(-500.0f, 10.0f, 0.0f).size() == 51, "axis -500:10:0 has 51 values");
    ok &= check(ifc_tools::TerrainAxis(0.0f, 10.0f, 25.0f).size() == 3, "axis stop off the lattice is truncated like MATLAB");
    ok &= check(throws([] { ifc_tools::TerrainAxis(0.0f, 0.0f, 10.0f); }), "rejects a zero step");
    ok &= check(throws([] { ifc_tools::TerrainAxis(0.0f, 10.0f, 5.0f); }), "rejects an axis with fewer than 2 values");
    ok &= check(throws([] { ifc_tools::TerrainAxis(std::nanf(""), 10.0f, 20.0f); }), "rejects a NaN axis");
    ok &= check(throws([] { ifc_tools::TerrainOrigin(91.0, 0.0, 0.0f); }), "rejects a latitude outside [-90, 90]");
    ok &= check(throws([] { ifc_tools::TerrainOrigin(0.0, 181.0, 0.0f); }), "rejects a longitude outside [-180, 180]");
    ok &= check(throws([] { ifc_tools::TerrainOrigin(0.0, 0.0, std::nanf("")); }), "rejects a NaN altitude");
    ok &= check(throws([] { ifc_tools::TerrainGrid(ifc_tools::TerrainOrigin(0.0, 0.0, 0.0f), ifc_tools::TerrainAxis(0.0f, 10.0f, 10.0f), ifc_tools::TerrainAxis(0.0f, 10.0f, 10.0f), std::vector<float>(3, 0.0f)); }), "rejects z of the wrong size");

    return ok ? 0 : 1;
}
