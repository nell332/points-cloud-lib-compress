// TimeTool.cpp
#include "TimeTool.h"

// 定义静态成员变量
std::chrono::high_resolution_clock::time_point Timer::start_time;

void Timer::Start() {
    start_time = std::chrono::high_resolution_clock::now();
}

double Timer::Stop() {
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
    return duration.count() / 1000000.0;
}