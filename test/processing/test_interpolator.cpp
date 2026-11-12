#include <ifc_tools/processing/interpolator.hpp>
#include <ifc_tools/processing/ring_buffer.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <optional>
#include <random>
#include <stdexcept>
#include <vector>

namespace
{

bool check(bool condition, const char* name)
{
    std::printf("%s: %s\n", condition ? "PASS" : "FAIL", name);
    return condition;
}

template <typename Timestamp, typename T>
struct Sample
{
    Timestamp timestamp;
    T value;
};

template <typename Timestamp>
class Linear : public ifc_tools::InterpolationMethod<Sample<Timestamp, double>>
{
public:
    Sample<Timestamp, double> interpolate(const Sample<Timestamp, double>& a, const Sample<Timestamp, double>& b, float alpha) const override
    {
        return {a.timestamp, a.value + static_cast<double>(alpha) * (b.value - a.value)};
    }
};

template <typename Container>
auto values(const Container& samples)
{
    std::vector<std::optional<decltype(samples.front()->value)>> out;

    for (const auto& sample : samples)
    {
        out.push_back(sample ? std::optional(sample->value) : std::nullopt);
    }

    return out;
}

template <typename Container>
std::vector<int64_t> timestamps(const Container& samples)
{
    std::vector<int64_t> out;

    for (const auto& sample : samples)
    {
        out.push_back(sample->timestamp);
    }

    return out;
}

bool near(double a, double b)
{
    return std::abs(a - b) < 1e-4;
}

}

int main()
{
    bool ok = true;
    const Linear<int64_t> linear;

    const std::vector<Sample<int64_t, int>> samples{{0, 10}, {100, 20}, {300, 30}};
    const ifc_tools::Interpolator nearest(samples);
    ok &= check(nearest.interpolate(40)->value == 10 && nearest.interpolate(50)->value == 20 && nearest.interpolate(60)->value == 20, "nearest is the default method");
    ok &= check(nearest.interpolate(0)->value == 10 && nearest.interpolate(100)->value == 20 && nearest.interpolate(300)->value == 30, "sample timestamps return the sample");
    ok &= check(!nearest.interpolate(-1) && !nearest.interpolate(301), "outside the time range nothing is returned");
    const std::vector<int64_t> ascending{-1, 0, 50, 250, 300, 301};
    const auto swept_nearest = nearest.interpolate(ascending);
    ok &= check(values(swept_nearest) == std::vector<std::optional<int>>{std::nullopt, 10, 20, 30, 30, std::nullopt}, "ascending vector of queries");
    const std::vector<int64_t> inside{0, 50, 250, 300};
    ok &= check(timestamps(nearest.interpolate(inside)) == inside, "results carry the query timestamps");
    ok &= check(nearest.interpolate(std::vector<int64_t>{}).empty(), "no queries give no results");

    const std::vector<Sample<int64_t, double>> line{{0, 0.0}, {100, 1.0}, {300, 5.0}};
    const ifc_tools::Interpolator interpolator(line);
    ok &= check(near(interpolator.interpolate(25, linear)->value, 0.25) && near(interpolator.interpolate(200, linear)->value, 3.0), "custom method interpolates between the bracketing samples");

    std::mt19937 rng(7);
    std::uniform_int_distribution<int64_t> time(-50, 350);
    std::vector<int64_t> queries(200);

    for (int64_t& t : queries)
    {
        t = time(rng);
    }

    std::sort(queries.begin(), queries.end());
    const auto swept = interpolator.interpolate(queries, linear);
    bool same = swept.size() == queries.size();

    for (std::size_t k = 0; same && k < queries.size(); ++k)
    {
        const auto single = interpolator.interpolate(queries[k], linear);
        same = swept[k].has_value() == single.has_value() && (!single || swept[k]->value == single->value);
    }

    ok &= check(same, "forward sweep matches independent single queries");

    const std::vector<Sample<int64_t, int>> single{{100, 7}};
    ok &= check(values(ifc_tools::Interpolator(single).interpolate(std::vector<int64_t>{0, 100, 200})) == std::vector<std::optional<int>>{std::nullopt, 7, std::nullopt},
        "a single sample is returned only at its timestamp");

    const std::vector<Sample<int64_t, int>> empty;
    bool thrown = false;

    try
    {
        ifc_tools::Interpolator(empty).interpolate(0);
    }
    catch (const std::out_of_range&)
    {
        thrown = true;
    }

    ok &= check(thrown, "empty container throws out_of_range");

    const std::vector<Sample<double, double>> seconds{{0.0, 0.0}, {0.5, 2.0}};
    ok &= check(near(ifc_tools::Interpolator(seconds).interpolate(0.25, Linear<double>())->value, 1.0), "timestamp type is deduced from the element");

    ifc_tools::RingBuffer<Sample<int64_t, double>> ring(2);

    for (const auto& sample : line)
    {
        ring.push(sample);
    }

    ok &= check(near(ifc_tools::Interpolator(ring).interpolate(200, linear)->value, 3.0) && !ifc_tools::Interpolator(ring).interpolate(0, linear),
        "a wrapped ring buffer can be decorated");

    return ok ? 0 : 1;
}
