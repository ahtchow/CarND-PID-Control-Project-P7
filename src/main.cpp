#include <math.h>
#include <uWS/uWS.h>
#include <iostream>
#include <string>
#include "json.hpp"
#include "PID.h"

// for convenience
using nlohmann::json;
using std::string;

static constexpr double tau_p = 0.09;
static constexpr double tau_i = 0.0004;
static constexpr double tau_d = 1.7;
static constexpr double THROTTLE = 0.3;
static const vector<double> INCREMENT = {0.000001, 0.000000001, 0.00001};
static const int MAX_STEPS = 7;

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

int main() {
  uWS::Hub h;

  //Initializing the PID Controller
  PID pid;
  pid.Init(tau_p,tau_i,tau_d);

  h.onMessage([&pid](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, 
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
          double cte = std::stod(j[1]["cte"].get<string>());
          double speed = std::stod(j[1]["speed"].get<string>());
          double angle = std::stod(j[1]["steering_angle"].get<string>());
          double steer_value;
          
          pid.UpdateError(cte);
          steer_value = pid.TotalError();
          //Update Params
          pid.Twiddle(INCREMENT, MAX_STEPS);

          std::cout
              << "\n======[" << "DASHBOARD" << "]======"
              << "\nCross Track Error:               " << cte
              << "\nCurrent Speed:                   " << speed
              << "\nCurrent Steering Angle:          " << angle
              << "\nPID Corrected Steering:          " << steer_value
              << "\nPID Steering Coefficients:       " <<   "Kp = " << pid.Kp_()
                                                       << ", Ki = " << pid.Ki_()
                                                       << ", Kd = " << pid.Kd_()
              << "\n======[" "DASHBOARD" "]======\n"
              << std::endl;

          pid.ExecuteUpdate(steer_value, THROTTLE, ws);
        }

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