#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "MPC.h"
#include "json.hpp"

// for convenience
using json = nlohmann::json;

const int    waypoint_points = 15;
const double waypoint_distance = 3.0;

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
  auto b2 = s.rfind("}]");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

// Evaluate a polynomial.
double polyeval(Eigen::VectorXd coeffs, double x) {
  double result = 0.0;
  for (int i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * pow(x, i);
  }
  return result;
}

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,
                        int order) {
  assert(xvals.size() == yvals.size());
  assert(order >= 1 && order <= xvals.size() - 1);
  Eigen::MatrixXd A(xvals.size(), order + 1);

  for (int i = 0; i < xvals.size(); i++) {
    A(i, 0) = 1.0;
  }

  for (int j = 0; j < xvals.size(); j++) {
    for (int i = 0; i < order; i++) {
      A(j, i + 1) = A(j, i) * xvals(j);
    }
  }

  auto Q = A.householderQr();
  auto result = Q.solve(yvals);
  return result;
}

int main() {
  uWS::Hub h;

  // MPC is initialized here!
  MPC mpc;

  h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    string sdata = string(data).substr(0, length);
    cout << sdata << endl;
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      string s = hasData(sdata);
      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          vector<double> ptsx = j[1]["ptsx"];
          vector<double> ptsy = j[1]["ptsy"];
          double px = j[1]["x"];
          double py = j[1]["y"];
          double psi = j[1]["psi"];
          double v = j[1]["speed"];
          int N = ptsx.size();

          /*
          * TODO: Calculate steeering angle and throttle using MPC.
          *
          * Both are in between [-1, 1].
          *
          */
          // Convert to the vehicle coordinate system
          Eigen::VectorXd ptsx_vc(N);
          Eigen::VectorXd ptsy_vc(N);
          for(int i = 0; i < N; i++) {
            ptsx_vc[i] = (ptsx[i] - px) * cos(psi) + (ptsy[i] - py) * sin(psi);
            ptsy_vc[i] = (ptsy[i] - py) * cos(psi) - (ptsx[i] - px) * sin(psi);
          }

          // TODO: fit a polynomial to the above x and y coordinates
          auto coeffs = polyfit(ptsx_vc, ptsy_vc, 3);

          // TODO: calculate the cross track error
          // double cte = polyeval(coeffs, 0) - py;
          double cte = polyeval(coeffs, 0.0);

          // TODO: calculate the orientation error
          double epsi = atan(coeffs[1]);
          // double epsi = -atan(coeffs[1]);
          // double epsi = atan(coeffs[1]+2*coeffs[2]*px);
          
          // Generate coordinate pairs to plot based on fit
          vector<double> next_x(waypoint_points);
          vector<double> next_y(waypoint_points);
          for (int k=0; k<waypoint_points; k++)
          {
            next_x[k] = waypoint_distance * (double)k;
            next_y[k] = polyeval(coeffs, next_x[k]);
          }

          // Incorporate any latency 
          double latency_dt = 0.1; // 100 ms
          double Lf = 2.67;
          double throttle = j[1]["throttle"];
          double steering_angle = j[1]["steering_angle"];

          double latency_x = v * latency_dt;
          double latency_y = 0;
          double latency_psi = -(v / Lf) * steering_angle * latency_dt;
          double latency_v = v + throttle * latency_dt;
          double latency_cte = cte + v * sin(epsi) * latency_dt;

          // Compute the expected heading based on coeffs.
          double expected_psi = atan(coeffs[1] + 
                                2.0 * coeffs[2] * latency_x + 
                                3.0 * coeffs[3] * latency_x*latency_x);

          // Compute the latent heading error.
          double latency_epsi = psi - expected_psi;
          
          // Compose the state to pass into the solver.
          Eigen::VectorXd state(6); // State has 6 elements
          state << latency_x, latency_y, latency_psi, latency_v, latency_cte, latency_epsi;
          // state << 0.0, 0.0, 0.0, v, cte, epsi;
          // state << px, py, psi, v, cte, epsi;

          auto vars = mpc.Solve(state, coeffs);
          // cout << "vars = " << vars[0] << ", " << vars[1] << ", " << vars[2] << ", " << vars[3];
          // cout << " vars = " << vars[4] << ", " << vars[5] << ", " << vars[6] << ", " << vars[7] << std::endl;

          double steer_value;
          double throttle_value;

          steer_value = -vars[6]; 
          throttle_value = vars[7];

          json msgJson;
          // NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
          // Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value;

          //Display the MPC predicted trajectory 
          // vector<double> mpc_x_vals;
          // vector<double> mpc_y_vals;

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Green line
          msgJson["mpc_x"] = mpc.mpc_x;
          msgJson["mpc_y"] = mpc.mpc_y;

          //Display the waypoints/reference line
          // vector<double> next_x_vals;
          // vector<double> next_y_vals;

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Yellow line

          msgJson["next_x"] = next_x;
          msgJson["next_y"] = next_y;

          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
          // Latency
          // The purpose is to mimic real driving conditions where
          // the car does actuate the commands instantly.
          //
          // Feel free to play around with this value but should be to drive
          // around the track with 100ms latency.
          //
          // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE
          // SUBMITTING.
          this_thread::sleep_for(chrono::milliseconds(100));
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

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
