#pragma once
#include <atomic>
#include <condition_variable>
#include <functional>
#include <limits>
#include <mutex>
#include <queue>

namespace dai {

template <typename T>
class LockingQueue {
   public:
    LockingQueue() = default;
    explicit LockingQueue(unsigned maxSize, bool blocking = true) : maxSize(maxSize), blocking(blocking) {}

    void setMaxSize(unsigned sz) {
        // Lock first
        std::unique_lock<std::mutex> lock(guard);
        maxSize = sz;
    }

    void setBlocking(bool bl) {
        // Lock first
        std::unique_lock<std::mutex> lock(guard);
        blocking = bl;
    }

    unsigned getMaxSize() const {
        // Lock first
        std::unique_lock<std::mutex> lock(guard);
        return maxSize;
    }

    bool getBlocking() const {
        // Lock first
        std::unique_lock<std::mutex> lock(guard);
        return blocking;
    }

    void destruct() {
        std::unique_lock<std::mutex> lock(guard);
        if(!destructed) {
            signalPop.notify_all();
            signalPush.notify_all();
            destructed = true;
        }
    }
    ~LockingQueue() = default;

    template <typename Rep, typename Period>
    bool waitAndConsumeAll(std::function<void(T&)> callback, std::chrono::duration<Rep, Period> timeout) {
        {
            std::unique_lock<std::mutex> lock(guard);

            // First checks predicate, then waits
            bool pred = signalPush.wait_for(lock, timeout, [this]() { return !queue.empty() || destructed; });
            if(!pred) return false;
            if(destructed) return false;

            // Continue here if and only if queue has any elements
            while(!queue.empty()) {
                callback(queue.front());
                queue.pop();
            }
        }

        signalPop.notify_all();
        return true;
    }

    bool waitAndConsumeAll(std::function<void(T&)> callback) {
        {
            std::unique_lock<std::mutex> lock(guard);

            signalPush.wait(lock, [this]() { return !queue.empty() || destructed; });
            if(queue.empty()) return false;
            if(destructed) return false;

            while(!queue.empty()) {
                callback(queue.front());
                queue.pop();
            }
        }

        signalPop.notify_all();
        return true;
    }

    bool consumeAll(std::function<void(T&)> callback) {
        {
            std::lock_guard<std::mutex> lock(guard);

            if(queue.empty()) return false;

            while(!queue.empty()) {
                callback(queue.front());
                queue.pop();
            }
        }

        signalPop.notify_all();
        return true;
    }

    bool push(T const& data) {
        {
            std::unique_lock<std::mutex> lock(guard);
            if(maxSize == 0) {
                // necessary if maxSize was changed
                while(!queue.empty()) {
                    queue.pop();
                }
                return true;
            }
            if(!blocking) {
                // if non blocking, remove as many oldest elements as necessary, so next one will fit
                // necessary if maxSize was changed
                while(queue.size() >= maxSize) {
                    queue.pop();
                }
            } else {
                signalPop.wait(lock, [this]() { return queue.size() < maxSize || destructed; });
                if(destructed) return false;
            }

            queue.push(data);
        }
        signalPush.notify_all();
        return true;
    }

    template <typename Rep, typename Period>
    bool tryWaitAndPush(T const& data, std::chrono::duration<Rep, Period> timeout) {
        {
            std::unique_lock<std::mutex> lock(guard);
            if(maxSize == 0) {
                // necessary if maxSize was changed
                while(!queue.empty()) {
                    queue.pop();
                }
                return true;
            }
            if(!blocking) {
                // if non blocking, remove as many oldest elements as necessary, so next one will fit
                // necessary if maxSize was changed
                while(queue.size() >= maxSize) {
                    queue.pop();
                }
            } else {
                // First checks predicate, then waits
                bool pred = signalPop.wait_for(lock, timeout, [this]() { return queue.size() < maxSize || destructed; });
                if(!pred) return false;
                if(destructed) return false;
            }

            queue.push(data);
        }
        signalPush.notify_all();
        return true;
    }

    bool empty() const {
        std::lock_guard<std::mutex> lock(guard);
        return queue.empty();
    }

    bool front(T& value) {
        std::unique_lock<std::mutex> lock(guard);
        if(queue.empty()) {
            return false;
        }

        value = queue.front();
        return true;
    }

    bool tryPop(T& value) {
        {
            std::lock_guard<std::mutex> lock(guard);
            if(queue.empty()) {
                return false;
            }

            value = std::move(queue.front());
            queue.pop();
        }
        signalPop.notify_all();
        return true;
    }

    bool waitAndPop(T& value) {
        {
            std::unique_lock<std::mutex> lock(guard);

            signalPush.wait(lock, [this]() { return (!queue.empty() || destructed); });
            if(queue.empty()) return false;
            if(destructed) return false;

            value = std::move(queue.front());
            queue.pop();
        }
        signalPop.notify_all();
        return true;
    }

    template <typename Rep, typename Period>
    bool tryWaitAndPop(T& value, std::chrono::duration<Rep, Period> timeout) {
        {
            std::unique_lock<std::mutex> lock(guard);

            // First checks predicate, then waits
            bool pred = signalPush.wait_for(lock, timeout, [this]() { return !queue.empty() || destructed; });
            if(!pred) return false;
            if(destructed) return false;

            value = std::move(queue.front());
            queue.pop();
        }
        signalPop.notify_all();
        return true;
    }

    void waitEmpty() {
        std::unique_lock<std::mutex> lock(guard);
        signalPop.wait(lock, [this]() { return queue.empty() || destructed; });
    }

   private:
    unsigned maxSize = std::numeric_limits<unsigned>::max();
    bool blocking = true;
    std::queue<T> queue;
    mutable std::mutex guard;
    bool destructed{false};
    std::condition_variable signalPop;
    std::condition_variable signalPush;
};

}  // namespace dai
