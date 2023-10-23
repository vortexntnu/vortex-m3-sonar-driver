#pragma once

#include <vector>
#include <mutex>
#include <condition_variable>

namespace m3 {
namespace utils {

template<typename T>
class RingBuffer {
public:
    explicit RingBuffer(size_t size);
    void put(T item);
    T get();
    void reset();
    bool is_empty() const;
    bool is_full() const;
    size_t capacity() const;
    size_t size() const;

private:
    std::vector<T> buffer;
    size_t head;
    size_t tail;
    const size_t max_size;
    bool full;
    mutable std::mutex mtx; // 'mutable' is used here to allow the modification of 'mtx' inside 'const' methods
    std::condition_variable cond;
};


// Implementation
// This may alternatively be defined in a .tpp file and included at the bottom of the header file
template<typename T>
RingBuffer<T>::RingBuffer(size_t size) : max_size(size), buffer(size), head(0), tail(0), full(false) {}

template<typename T>
void RingBuffer<T>::put(T item) {
    std::unique_lock<std::mutex> lock(mtx);
    cond.wait(lock, [this](){ return !full; }); // Wait if the buffer is full

    buffer[head] = item;
    if(full) {
        tail = (tail + 1) % max_size;
    }

    head = (head + 1) % max_size;

    full = head == tail;
    lock.unlock();
    cond.notify_all(); // Notify the consumer that data is available
}

template<typename T>
T RingBuffer<T>::get() {
    std::unique_lock<std::mutex> lock(mtx);
    cond.wait(lock, [this](){ return !is_empty(); }); // Wait if the buffer is empty

    // Read data and move the tail forward
    auto val = buffer[tail];
    full = false;
    tail = (tail + 1) % max_size;

    lock.unlock();
    cond.notify_all(); // Notify the producer that space is available

    return val;
}

template<typename T>
void RingBuffer<T>::reset() {
    std::lock_guard<std::mutex> lock(mtx);
    head = tail;
    full = false;
}

template<typename T>
bool RingBuffer<T>::is_empty() const {
    return (!full && (head == tail));
}

template<typename T>
bool RingBuffer<T>::is_full() const {
    return full;
}

template<typename T>
size_t RingBuffer<T>::capacity() const {
    return max_size;
}

template<typename T>
size_t RingBuffer<T>::size() const {
    size_t size = max_size;

    if(!full) {
        if(head >= tail) {
            size = head - tail;
        } else {
            size = max_size + head - tail;
        }
    }

    return size;
}

} // namespace utils
} // namespace m3