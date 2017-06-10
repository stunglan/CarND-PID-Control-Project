#include "PID.h"

using namespace std;

#include <chrono>
#include <ctime>

/*
* TODO: Complete the PID class.
*/

PID::PID() {
    /*
     * Coefficients
     */
    this->Kp = 0;
    this->Ki = 0;
    this->Kd = 0;
    
    p_error = 0;
    i_error  = 0;
    d_error = 0;
    
    cte_prev = 0;
    

}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
    
    /*
     * Coefficients
     */
    this->Kp = Kp;
    this->Ki = Ki;
    this->Kd = Kd;
    
    prev_time = std::chrono::system_clock::now();
    

    
}

void PID::UpdateError(double cte) {
    
    std::chrono::time_point<std::chrono::system_clock> t;
    
    t = std::chrono::system_clock::now();
    
    std::chrono::duration<double> elapsed_seconds = (t-prev_time);
    // use formulas for PID given in lectures
    p_error = cte;
    i_error += cte;
    d_error = (cte - cte_prev)/elapsed_seconds.count();
    
    // keep last error
    cte_prev = p_error;
    prev_time = t;
    
}

double PID::TotalError() {
    
    // total
    return -1*(Kp * p_error + Kd * d_error + Ki * i_error);
    
}

