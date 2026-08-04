#pragma once

#include <condition_variable>
#include <cstddef>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <utility>
#include <vector>

namespace visionarm {

// Fixed-capacity ring queue. Storage is allocated once in the constructor;
// push/pop operations do not allocate linked-list nodes.
template <typename T>
class BoundedQueue final {
public:
    explicit BoundedQueue(std::size_t capacity)
        : slots_(capacity) {
        if (capacity == 0U) {
            throw std::invalid_argument("BoundedQueue capacity must be > 0");
        }
    }

    BoundedQueue(const BoundedQueue&) = delete;
    BoundedQueue& operator=(const BoundedQueue&) = delete;

    bool TryPush(T value) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (stopped_ || size_ == slots_.size()) {
            return false;
        }
        PushUnlocked(std::move(value));
        not_empty_.notify_one();
        return true;
    }

    bool WaitPush(T value) {
        std::unique_lock<std::mutex> lock(mutex_);
        not_full_.wait(lock, [this] {
            return stopped_ || size_ < slots_.size();
        });
        if (stopped_) {
            return false;
        }
        PushUnlocked(std::move(value));
        lock.unlock();
        not_empty_.notify_one();
        return true;
    }

    // Keeps the newest item. When full, removes the oldest queued item and
    // returns it through evicted so the caller can release external resources
    // such as a dequeued V4L2 buffer.
    bool PushLatest(T value, std::optional<T>* evicted) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (evicted != nullptr) {
            evicted->reset();
        }

        if (stopped_) {
            if (evicted != nullptr) {
                *evicted = std::move(value);
            }
            return false;
        }

        if (size_ == slots_.size()) {
            if (evicted != nullptr) {
                *evicted = std::move(*slots_[head_]);
            }
            slots_[head_].reset();
            head_ = Next(head_);
            --size_;
        }

        PushUnlocked(std::move(value));
        not_empty_.notify_one();
        return true;
    }

    bool TryPop(T* value) {
        if (value == nullptr) {
            return false;
        }
        std::lock_guard<std::mutex> lock(mutex_);
        if (size_ == 0U) {
            return false;
        }
        PopUnlocked(value);
        not_full_.notify_one();
        return true;
    }

    // After Stop(), queued items are still drained. Returns false only when the
    // queue is both stopped and empty.
    bool WaitPop(T* value) {
        if (value == nullptr) {
            return false;
        }
        std::unique_lock<std::mutex> lock(mutex_);
        not_empty_.wait(lock, [this] {
            return stopped_ || size_ > 0U;
        });
        if (size_ == 0U) {
            return false;
        }
        PopUnlocked(value);
        lock.unlock();
        not_full_.notify_one();
        return true;
    }

    void Stop() noexcept {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            stopped_ = true;
        }
        not_empty_.notify_all();
        not_full_.notify_all();
    }

    [[nodiscard]] std::size_t Size() const noexcept {
        std::lock_guard<std::mutex> lock(mutex_);
        return size_;
    }

    [[nodiscard]] std::size_t Capacity() const noexcept {
        return slots_.size();
    }

private:
    [[nodiscard]] std::size_t Next(std::size_t index) const noexcept {
        return (index + 1U) % slots_.size();
    }

    void PushUnlocked(T value) {
        slots_[tail_] = std::move(value);
        tail_ = Next(tail_);
        ++size_;
    }

    void PopUnlocked(T* value) {
        *value = std::move(*slots_[head_]);
        slots_[head_].reset();
        head_ = Next(head_);
        --size_;
    }

    std::vector<std::optional<T>> slots_;
    std::size_t head_ = 0U;
    std::size_t tail_ = 0U;
    std::size_t size_ = 0U;
    bool stopped_ = false;

    mutable std::mutex mutex_;
    std::condition_variable not_empty_;
    std::condition_variable not_full_;
};

}  // namespace visionarm
