#include <math.h>
#include <uWS/uWS.h>
#include <iostream>
#include <string>
#include "json.hpp"
#include "PID.h"
#include "PIDAutotune.h"

// for convenience
using nlohmann::json;
using std::string;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != string::npos) {
    return "";
  }
  else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

int main(int argc, char** argv) {
  uWS::Hub h;

  bool tune_pid = false;

  PID steering_pid;
  PID throttle_pid;

  /**
   * TODO: Initialize the pid variable.
   */
  //steering_pid.Init(0.2, 0., 2.);
  if(argc < 2) {
    steering_pid.Init(0.04, 0., .2);
  } else {
    std::string::size_type sz;
    double Kp = std::stod(argv[1], &sz);
    double Ki = std::stod(argv[2], &sz);
    double Kd = std::stod(argv[3], &sz);
    steering_pid.Init(Kp, Ki, Kd);
  }
  //steering_pid.Init(0,0,0);
  //throttle_pid.Init(0.3, 0.0005, 0.02);
  throttle_pid.Init(0.1, 0.0001, 1);

  PIDAutotune auto_tuner(&steering_pid);

  h.onMessage([&steering_pid, &throttle_pid, &auto_tuner, &tune_pid](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {
      auto s = hasData(string(data).substr(0, length));

      if (s != "") {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte_steering = std::stod(j[1]["cte"].get<string>());
          double speed = std::stod(j[1]["speed"].get<string>());
          double angle = std::stod(j[1]["steering_angle"].get<string>());
          double steer_value;
          double throttle_value;


          /**
           * TODO: Calculate steering value here, remember the steering value is
           *   [-1, 1].
           * NOTE: Feel free to play around with the throttle and speed.
           *   Maybe use another PID controller to control the speed!
           */

          // steering controller
          steering_pid.UpdateError(cte_steering);
          steer_value = steering_pid.Control(-1., 1.);

          double target_speed = 30. * (1. - abs(steer_value)) + 30.;
          double cte_speed = speed - target_speed;

          // throttle controller
          throttle_pid.UpdateError(cte_speed);
          throttle_value = throttle_pid.Control(-1., 1.);

          // DEBUG
          /*
          std::cout << "*** Steering: " << std::endl;
          std::cout << "CTE: " << cte_steering << " Steering Value: " << steer_value
                    << std::endl;

          std::cout << "*** Speed: " << std::endl;
          std::cout << "CTE: " << cte_speed << " Throttle Value: " << throttle_value
                    << std::endl;
          */

          if(tune_pid == true) {
            auto_tuner.addUpError(cte_steering);

            if((auto_tuner.didFinishRun() == true) || (auto_tuner.cteIndicatesOffTrack(cte_steering) == true)) {
              std::cout << "Twiddeling!" << std::endl;

              if(auto_tuner.twiddle(0.00001) == true) {
                std::cout << "********** FINISHED TWIDDELING **********" << std::endl;
              } else {
                // reset the simulator
                std::string msg("42[\"reset\", {}]");
                ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
              }
            }
          }

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          //std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket message if
  }); // end h.onMessage

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }

  h.run();
}
