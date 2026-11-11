#pragma once

#include <cstddef>
#include <cstdint>
#include <iterator>
#include <stdexcept>
#include <utility>
#include <vector>

namespace ifc_tools
{

template <typename T>
class RingBuffer;

template <typename T>
class RingBufferConstIterator
{
public:
    using iterator_category = std::forward_iterator_tag;
    using value_type = T;
    using difference_type = std::ptrdiff_t;
    using pointer = const T*;
    using reference = const T&;

    RingBufferConstIterator() = default;

    RingBufferConstIterator(const RingBuffer<T>* buffer, std::size_t index)
        : _buffer(buffer), _index(index)
    {
    }

    const T& operator*() const
    {
        return (*_buffer)[_index];
    }

    RingBufferConstIterator& operator++()
    {
        ++_index;
        return *this;
    }

    RingBufferConstIterator operator++(int)
    {
        RingBufferConstIterator copy = *this;
        ++_index;
        return copy;
    }

    bool operator==(const RingBufferConstIterator&) const = default;

private:
    const RingBuffer<T>* _buffer = nullptr;
    std::size_t _index = 0;
};

template <typename T>
class RingBuffer
{
public:
    explicit RingBuffer(std::size_t capacity)
        : _capacity(capacity)
    {
        if (capacity == 0)
        {
            throw std::invalid_argument("RingBuffer capacity must be positive");
        }

        _data.reserve(capacity);
    }

    void push(T value)
    {
        if (_data.size() < _capacity)
        {
            _data.push_back(std::move(value));
            return;
        }

        _data[_head] = std::move(value);
        _head = (_head + 1) % _capacity;
    }

    const T& operator[](std::size_t index) const
    {
        return _data[(_head + index) % _capacity];
    }

    const T& front() const
    {
        return (*this)[0];
    }

    const T& back() const
    {
        return (*this)[_data.size() - 1];
    }

    std::size_t size() const
    {
        return _data.size();
    }

    bool empty() const
    {
        return _data.empty();
    }

    void clear()
    {
        _data.clear();
        _head = 0;
    }

    RingBufferConstIterator<T> begin() const
    {
        return RingBufferConstIterator<T>(this, 0);
    }

    RingBufferConstIterator<T> end() const
    {
        return RingBufferConstIterator<T>(this, _data.size());
    }

    std::vector<T> toVector() const
    {
        return std::vector<T>(begin(), end());
    }

private:
    std::vector<T> _data;
    std::size_t _capacity;
    std::size_t _head = 0;
};

}
