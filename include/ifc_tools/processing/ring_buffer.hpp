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

        _data.resize(capacity);
    }

    void push(T value)
    {
        if (_size < _capacity)
        {
            _data[(_head + _size) % _capacity] = std::move(value);
            ++_size;
            return;
        }

        _data[_head] = std::move(value);
        _head = (_head + 1) % _capacity;
    }

    void popFront()
    {
        if (_size == 0)
        {
            throw std::out_of_range("RingBuffer is empty");
        }

        _data[_head] = T();
        _head = (_head + 1) % _capacity;
        --_size;
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
        return (*this)[_size - 1];
    }

    std::size_t size() const
    {
        return _size;
    }

    bool empty() const
    {
        return _size == 0;
    }

    void clear()
    {
        _head = 0;
        _size = 0;
    }

    RingBufferConstIterator<T> begin() const
    {
        return RingBufferConstIterator<T>(this, 0);
    }

    RingBufferConstIterator<T> end() const
    {
        return RingBufferConstIterator<T>(this, _size);
    }

    std::vector<T> toVector() const
    {
        return std::vector<T>(begin(), end());
    }

private:
    std::vector<T> _data;
    std::size_t _capacity;
    std::size_t _head = 0;
    std::size_t _size = 0;
};

}
