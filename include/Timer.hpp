#pragma once
#include <chrono>

class Timer {
private:
    std::chrono::time_point<std::chrono::high_resolution_clock> start_time;
public:
    Timer() {
        reset();
    }

    void reset() {
        start_time = std::chrono::high_resolution_clock::now();
    }

    double elasped_microseconds() const {
        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
        return static_cast<double>(duration.count());
    }
};