#ifndef ONLINE_ITER_TIMER_H_
#define ONLINE_ITER_TIMER_H_

#include <chrono>
#include <ratio>
#include <iostream>
#include <string>

//class for general timer
namespace online_planner{
class Timer{
public:
    Timer():Timer(std::string("")){}
    Timer(std::string str_):num_tick(0), total_musec(0.0), prefix_(str_){}
    void tic(){
        ++num_tick;
        tic_ = std::chrono::high_resolution_clock::now();
    }    
    void toc(){
        std::chrono::high_resolution_clock::time_point toc_ = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double, std::micro> musec = toc_ - tic_;
        total_musec += musec.count();
    }
    void reset(){
        num_tick = 0;
        total_musec = 0.0;
    }
    void print(bool print_total = true, bool print_avg = true) const{
        std::cout<<prefix_<<std::endl;
        if(print_total) std::cout<<"Total : "<<total_musec<<"mu sec for "<<num_tick<<" operations"<<std::endl;
        if(print_avg && num_tick > 0) std::cout<<total_musec / num_tick<<" mu sec per operation"<<std::endl;
    }
protected:
    int num_tick;
    double total_musec;
    std::chrono::high_resolution_clock::time_point tic_;
    std::string prefix_;
};
}//namespace online_planner

#endif