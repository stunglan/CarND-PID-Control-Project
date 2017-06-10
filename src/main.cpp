#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include <math.h>

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != std::string::npos) {
    return "";
  }
  else if (b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

void move_car(uWS::WebSocket<uWS::SERVER> ws,double steer_value, double throttle){
  json msgJson;
  msgJson["steering_angle"] = steer_value;
  msgJson["throttle"] = throttle;
  auto msg = "42[\"steer\"," + msgJson.dump() + "]";
//  std::cout << msg << std::endl;
  ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
}


/*
 * Support class for my twiddle implementation
 */

class Twiddle {
public:
  bool mode = false;
  int iterator = 0;
  int parameter = 0;
  int length = 600;
  double tolerance = 0.2;
  double sensitivity = 0.1;
  double error = 0;
  double best_error = 999999999.9;
  double K[3] = {1.0,1.0,1.0};
  double bestK[3] = {1.0,1.0,1.0};
  double deltaK[3] = {1.0,1.0,1.0};
  bool initial_run = true;
  bool turned = false;

  
  Twiddle(double Kp, double Ki, double Kd) {
    this->K[0] = Kp;
    this->K[1] = Ki;
    this->K[2] = Kd;
    this->deltaK[0] = Kp/10.0;
    this->deltaK[1] = Ki/10.0;
    this->deltaK[2] = Kd/10.0;
    
    
    
  }
  void updateBestError(double error,double K[3]){
    best_error = error;
    bestK[0] = K[0];
    bestK[1] = K[1];
    bestK[2] = K[2];
  }
  void setToBestParameters(){
    
    K[0] = bestK[0];
    K[1] = bestK[1];
    K[2] = bestK[2];
  }
  
  
  double sumParameters() {
    return deltaK[0]+deltaK[1]+deltaK[2];
  }
  
  
};



int main()
{
  uWS::Hub h;
  

  PID pid = PID(); // call the constructor
  // TODO: Initialize the pid variable.
  
//  1st run double K[3] = {0.1,0.00004,0.003};
//  ENDING with Twiddle Kp: 7.73225 Ki: 0.0225684 Kd: 1.13507 Sum parameter: 0.198593
//  Best  1 Kp: 7.73225 Ki: 4e-05 Kd: 1.13507
//  double deltaK[3] = {1.0,1.0,1.0};
//  error all the way from start
//  3 hours
//  lenght 600
//  2dn run
//  ENDING with Twiddle Delta Kp: 0.171635 Ki: 6.60416e-07 Kd: 0.027995 Sum parameter: 0.199631
//  Best  1 Kp: 8.35383 Ki: 4.13947e-05 Kd: 1.02281
//  lenght 600
  
  
  double K[3] = {8.35383,4.13947e-05,1.02281};
  pid.Init(K[0], K[1], K[2]);
  
  Twiddle twiddle = Twiddle(K[0], K[1], K[2]);

  

  h.onMessage([&pid,&twiddle](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event


    
    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {
      auto s = hasData(std::string(data).substr(0, length));
      if (s != "") {
        auto j = json::parse(s);
        std::string event = j[0].get<std::string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<std::string>());
          double speed = std::stod(j[1]["speed"].get<std::string>());
          double angle = std::stod(j[1]["steering_angle"].get<std::string>());
          double steer_value;
          double throttle = 0.2;
          /*
          * TODO: Calcuate steering value here, remember the steering value is
          * [-1, 1].
          * NOTE: Feel free to play around with the throttle and speed. Maybe use
          * another PID controller to control the speed!
          */
          
          pid.UpdateError(cte);
    
          steer_value = pid.TotalError();
          
          // max speed 60
          if (speed > 60.0) {
            throttle = 0.0;
          }
          
            
          // DEBUG
          
//          std::cout << "CTE: " << cte << " Calc Steering: " << steer_value << " Sensor angle: " << angle << " Sensor speed: " << speed << std::endl;

          //
          // twiddle_mode -- implement twiddle 
          if (twiddle.mode) {
            
            
            twiddle.error += pow(cte,2.0);
            
            
            twiddle.iterator += 1;
            if (twiddle.iterator > twiddle.length) {
              std::cout  << "Error: " << twiddle.error / twiddle.iterator << " best_error: " << twiddle.best_error << std::endl;
              twiddle.error = twiddle.error / twiddle.length;
              
//              std::cout << "Twiddle Kp: " << twiddle.K[0] << " Ki: " << twiddle.K[1] << " Kd: " << twiddle.K[2] << std::endl;
              if (twiddle.initial_run) {
                twiddle.updateBestError(twiddle.error,twiddle.K);
                std::cout  << "Initial Error: " << twiddle.error << std::endl;

                twiddle.initial_run = false;
                twiddle.K[twiddle.parameter] += twiddle.deltaK[twiddle.parameter];
              }
              else {
                
                if (twiddle.error < twiddle.best_error) {
                  std::cout  << "New BEST Error: " << twiddle.error << std::endl;
                  twiddle.updateBestError(twiddle.error,twiddle.K);
                  twiddle.deltaK[twiddle.parameter] = twiddle.deltaK[twiddle.parameter] + (twiddle.deltaK[twiddle.parameter]*twiddle.sensitivity);
                  twiddle.parameter = (twiddle.parameter + 1) % 3;
                  twiddle.K[twiddle.parameter] =  twiddle.K[twiddle.parameter] + twiddle.deltaK[twiddle.parameter];
                  twiddle.turned = false;
                }
                else {
                  if (!twiddle.turned) {
                    twiddle.K[twiddle.parameter] = twiddle.K[twiddle.parameter] - 2.0*twiddle.deltaK[twiddle.parameter];
                    std::cout  << "Turn: " << twiddle.K[twiddle.parameter] << std::endl;
                    twiddle.turned = true;
                  }
                  else {
                    std::cout  << "Decrease parameter : " << twiddle.parameter  << " from " << twiddle.deltaK[twiddle.parameter] ;
                    twiddle.deltaK[twiddle.parameter] = twiddle.deltaK[twiddle.parameter] - twiddle.deltaK[twiddle.parameter]*twiddle.sensitivity;
                     std::cout  << " to " << twiddle.deltaK[twiddle.parameter] << std::endl;
                    twiddle.turned = false;
                    // start a new round
                    twiddle.setToBestParameters();
                    twiddle.parameter = (twiddle.parameter + 1) % 3;
                    twiddle.K[twiddle.parameter] += twiddle.deltaK[twiddle.parameter];
                  }

                }
              }
              
              
              // start the simulator over again with new parameters
              twiddle.iterator = 0;
              std::string msg("42[\"reset\", {}]");
              ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
              
              
              pid.Init(twiddle.K[0], twiddle.K[1], twiddle.K[2]);
              twiddle.error = 0;
              std::cout << std::endl << std::endl << "Twiddle parameter: " << twiddle.parameter << " Kp: " << twiddle.K[0] << " Ki: " << twiddle.K[1] << " Kd: " << twiddle.K[2] << std::endl;
              std::cout << "             Delta " << twiddle.parameter << " Kp: " << twiddle.deltaK[0] << " Ki: " << twiddle.deltaK[1] << " Kd: " << twiddle.deltaK[2] << " Sum parameter: " << twiddle.sumParameters() << std::endl;
              std::cout << "             Best  " << twiddle.parameter << " Kp: " << twiddle.bestK[0] << " Ki: " << twiddle.bestK[1] << " Kd: " << twiddle.bestK[2] << std::endl;
              
              // stop if we reach the max level
              if (twiddle.sumParameters() < twiddle.tolerance) {
                std::cout << "ENDING with Twiddle Delta Kp: " << twiddle.deltaK[0] << " Ki: " << twiddle.deltaK[1] << " Kd: " << twiddle.deltaK[2] << " Sum parameter: " << twiddle.sumParameters()<<std::endl;
                std::cout << "             Best  " << " Kp: " << twiddle.bestK[0] << " Ki: " << twiddle.bestK[1] << " Kd: " << twiddle.bestK[2] << std::endl;
                exit(0);
              }

              
              
            }
            
          }
         
          
          //
          move_car(ws,steer_value,throttle);
          
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1)
    {
      res->end(s.data(), s.length());
    }
    else
    {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
    // start the simulator over again with new parameters

  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port))
  {
    std::cout << "Listening to port " << port << std::endl;
  }
  else
  {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
