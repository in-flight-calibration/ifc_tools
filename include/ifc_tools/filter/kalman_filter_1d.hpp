#pragma once

#include <algorithm>
#include <cmath>

namespace ifc_tools
{

template<typename T>
class KalmanFilter1D
{
public:
    struct Parameters
    {
        T dQdt = 0.0;
        T innov_gate_sd = 3.0;
    };

    struct State
    {
        T x = 0.0;
        T P = 0.0;
    };

    explicit KalmanFilter1D(const Parameters& params)
        : _params(params)
    {
    }

    void initialize(T x, T P)
    {
        _state.x = x;
        _state.P = P;
        _initialized = true;
    }

    bool initialized() const
    {
        return _initialized;
    }

    bool predict(T dt)
    {
        if (!_initialized)
        {
            return false;
        }

        _state.P += _params.dQdt * std::max(dt, T(0));

        return true;
    }

    bool update(T z, T R)
    {
        if (!_initialized)
        {
            return false;
        }

        const T innov = z - _state.x;
        const T innov_var = _state.P + R;
        const T norm_innov = innov / std::sqrt(innov_var);

        if (std::abs(norm_innov) > _params.innov_gate_sd)
        {
            return false;
        }

        const T K = _state.P / innov_var;
        _state.x += K * innov;
        _state.P *= 1.0 - K;

        return true;
    }

    const State& state() const
    {
        return _state;
    }

private:
    const Parameters _params;

    bool _initialized = false;
    State _state;
};

}
