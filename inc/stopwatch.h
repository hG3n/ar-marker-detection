#ifndef STOPWATCH_H
#define STOPWATCH_H

#include <chrono>

struct Stopwatch
{
    static std::chrono::steady_clock::time_point getStart() {
        return std::chrono::steady_clock::now();
    }

    static double getElapsed(const std::chrono::steady_clock::time_point &start) {
        auto now = std::chrono::steady_clock::now();
        return (std::chrono::duration_cast<std::chrono::microseconds>( now- start).count()) / 1000.0;
    }
};

#endif // STOPWATCH_H
