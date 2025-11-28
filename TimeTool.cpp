// TimeTool.cpp
#include "TimeTool.h"

void Timer::Start() {
    start_time = std::chrono::high_resolution_clock::now();
}

double Timer::Stop() {
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
    return duration.count() / 1000000.0;
}