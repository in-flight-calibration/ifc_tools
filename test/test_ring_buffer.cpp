#include <ifc_tools/ring_buffer.hpp>

#include <cstdio>
#include <vector>

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
    ifc_tools::RingBuffer<int> buffer(3);
    ok &= check(buffer.empty(), "starts empty");

    for (int i = 1; i <= 5; ++i)
    {
        buffer.push(i);
    }

    ok &= check(buffer.size() == 3, "stays at capacity after wrapping");
    ok &= check(buffer.toVector() == std::vector<int>{3, 4, 5}, "iterates newest values oldest first");
    ok &= check(buffer[0] == 3 && buffer[1] == 4 && buffer[2] == 5, "index after wrap");

    return ok ? 0 : 1;
}
