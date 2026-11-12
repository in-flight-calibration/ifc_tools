#pragma once

#include <cstdint>
#include <iterator>
#include <optional>
#include <stdexcept>
#include <type_traits>
#include <vector>

namespace ifc_tools
{

template <typename T>
class InterpolationMethod
{
public:
    virtual ~InterpolationMethod() = default;
    virtual T interpolate(const T& a, const T& b, float alpha) const = 0;
};

template <typename T>
class Nearest : public InterpolationMethod<T>
{
public:
    T interpolate(const T& a, const T& b, float alpha) const override
    {
        return alpha < 0.5f ? a : b;
    }
};

template <typename Container>
class Interpolator
{
public:
    using T = typename std::iterator_traits<decltype(std::begin(std::declval<const Container&>()))>::value_type;
    using Timestamp = std::remove_cv_t<decltype(T::timestamp)>;

    explicit Interpolator(const Container& samples)
        : _samples(samples)
    {
    }

    Interpolator(Container&&) = delete;

    std::optional<T> interpolate(Timestamp timestamp, const InterpolationMethod<T>& method = Nearest<T>()) const
    {
        return interpolate(std::vector<Timestamp>{timestamp}, method)[0];
    }

    std::vector<std::optional<T>> interpolate(
        const std::vector<Timestamp>& timestamps,
        const InterpolationMethod<T>& method = Nearest<T>()) const
    {
        if (_samples.size() == 0)
        {
            throw std::out_of_range("Interpolator: no samples");
        }

        std::vector<std::optional<T>> out;
        out.reserve(timestamps.size());
        const auto last = std::end(_samples);
        auto a = std::begin(_samples);
        auto b = a;

        if (_samples.size() > 1)
        {
            ++b;
        }

        for (const Timestamp t : timestamps)
        {
            while ((*b).timestamp < t)
            {
                auto next = b;
                ++next;

                if (next == last)
                {
                    break;
                }

                a = b;
                b = next;
            }

            const Timestamp ta = (*a).timestamp;
            const Timestamp tb = (*b).timestamp;

            if (t == ta)
            {
                out.push_back(*a);
            }
            else if (t == tb)
            {
                out.push_back(*b);
            }
            else if (ta < t && t < tb)
            {
                out.push_back(method.interpolate(*a, *b, static_cast<float>(t - ta) / static_cast<float>(tb - ta)));
            }
            else
            {
                out.push_back(std::nullopt);
                continue;
            }

            out.back()->timestamp = t;
        }

        return out;
    }

private:
    const Container& _samples;
};

}
