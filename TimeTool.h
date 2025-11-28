// TimeTool.h
#ifndef TIMETOOL_H_INCLUDED
#define TIMETOOL_H_INCLUDED

#include <chrono>

class Timer {
private:
    static std::chrono::high_resolution_clock::time_point start_time;
    
public:
    static void Start();
    static double Stop(); // ????
};

#endif // TIMETOOL_H_INCLUDED