// TimeTool.h
#ifndef TIMETOOL_H_INCLUDED
#define TIMETOOL_H_INCLUDED

#include <chrono>

class Timer {
private:
    std::chrono::high_resolution_clock::time_point start_time;
    
public:
    void Start();
    double Stop(); // ∑µªÿ√Î ˝
};

#endif // TIMETOOL_H_INCLUDED