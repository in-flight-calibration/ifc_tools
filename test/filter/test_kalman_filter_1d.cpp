#include <ifc_tools/filter/kalman_filter_1d.hpp>

#include <cmath>
#include <cstdio>

namespace
{

bool check(bool condition, const char* name)
{
    std::printf("%s: %s\n", condition ? "PASS" : "FAIL", name);
    return condition;
}

}

int main()
{
    bool ok = true;
    ifc_tools::KalmanFilter1D<double> filter({.dQdt = 2.0, .innov_gate_sd = 3.0});
    ok &= check(!filter.initialized(), "starts uninitialized");
    ok &= check(!filter.predict(1.0) && !filter.update(1.0, 1.0), "predict and update fail before initialize");

    filter.initialize(10.0, 4.0);
    ok &= check(filter.predict(0.5), "predict succeeds after initialize");
    ok &= check(filter.state().x == 10.0 && filter.state().P == 5.0, "predict keeps x and adds dQdt*dt to P");

    filter.predict(-1.0);
    ok &= check(filter.state().P == 5.0, "negative dt does not shrink P");

    ok &= check(filter.update(14.0, 5.0), "accepts a measurement inside the gate");
    ok &= check(filter.state().x == 12.0 && filter.state().P == 2.5, "update with equal variances halves the gap and P");

    ok &= check(!filter.update(12.0 + 3.1 * std::sqrt(3.5), 1.0), "rejects a measurement outside the gate");
    ok &= check(filter.state().x == 12.0 && filter.state().P == 2.5, "rejected measurement leaves the state unchanged");

    ifc_tools::KalmanFilter1D<double> constant({.dQdt = 0.0, .innov_gate_sd = 3.0});
    constant.initialize(0.0, 100.0);

    for (int i = 0; i < 100; ++i)
    {
        constant.update(7.0 + ((i % 2) != 0 ? 1.0 : -1.0), 1.0);
    }

    ok &= check(std::abs(constant.state().x - 7.0) < 0.05 && constant.state().P < 0.011, "converges on a constant value");

    ifc_tools::KalmanFilter1D<float> single({.dQdt = 1.0f, .innov_gate_sd = 3.0f});
    single.initialize(1.0f, 1.0f);
    single.predict(1.0f);
    ok &= check(single.update(2.0f, 2.0f) && single.state().x == 1.5f, "works with float");

    return ok ? 0 : 1;
}
