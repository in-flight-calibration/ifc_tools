#include <ifc_tools/processing/ring_buffer.hpp>

#include <cstdio>
#include <stdexcept>
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

    buffer.popFront();
    ok &= check(buffer.size() == 2 && buffer.front() == 4, "pop front removes oldest");

    buffer.push(6);
    buffer.push(7);
    ok &= check(buffer.toVector() == std::vector<int>{5, 6, 7}, "push after pop front wraps");

    buffer.popFront();
    buffer.popFront();
    buffer.popFront();
    ok &= check(buffer.empty(), "pop front to empty");

    bool thrown = false;

    try
    {
        buffer.popFront();
    }
    catch (const std::out_of_range&)
    {
        thrown = true;
    }

    ok &= check(thrown, "pop front on empty throws");

    return ok ? 0 : 1;
}
