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
#include <unistd.h>
#include <time.h>

// for convenience
using json = nlohmann::json;

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

Eigen::MatrixXd TransformGlobleToLocal(double x, double y, double psi, const vector<double> &ptsx, const vector<double> &ptsy) {
	assert(ptsx.size() == ptsy.size());
	int len = ptsx.size();
	Eigen::MatrixXd waypoints(2, len);
	for (int i = 0; i < len; i++) {
		//waypoints(0, i) = (ptsx[i] - x) / cos(psi) + (ptsy[i] - y)*sin(psi);
		//waypoints(1, i) = (ptsy[i] - tan(psi)*(ptsx[i] - x))*cos(psi);
		waypoints(0, i) = cos(psi) * (ptsx[i] - x) + sin(psi) * (ptsy[i] - y);
		waypoints(1, i) = -sin(psi) * (ptsx[i] - x) + cos(psi) * (ptsy[i] - y);
	}
	return waypoints;

}


int main() {
  uWS::Hub h;

  // MPC is initialized here!
  MPC mpc;
  int N = 10;
  int count = 1;
  double durationTime = 0;

  h.onMessage([&mpc,&N,&count, &durationTime](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    string sdata = string(data).substr(0, length);
    //cout << sdata << endl;
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
		  double a= j[1]["throttle"];
		  double delta = j[1]["steering_angle"];
		 //delta = delta * deg2rad(25); as already done in the simulator

          /*
          * TODO: Calculate steering angle and throttle using MPC.
          *
          * Both are in between [-1, 1].
          *
          */
		  Eigen::MatrixXd waypoints = TransformGlobleToLocal(px, py, psi, ptsx, ptsy);
		  Eigen::VectorXd local_ptsx = waypoints.row(0);
		  Eigen::VectorXd local_ptsy = waypoints.row(1);

		  // fit polyniminal
		  Eigen::VectorXd coeffs = polyfit(local_ptsx, local_ptsy, 3);
		  double cte = polyeval(coeffs, 0);
		  double epsi = -atan(coeffs[1]); // x=0 position
		  
		 //consoder delay 0.1
		  double delay_t = 0.1;
		  double x_delay = 0 + v * cos(psi) * delay_t;
		  double y_delay = 0 + v * sin(psi) * delay_t;
		  double psi_delay = 0 - v * delta / 2.67 * delay_t; //modify turn direction as mentioned 
          double v_delay = v + a * delay_t;
          double cte_delay =(cte - 0) + v * sin(epsi)*delay_t;
          double epsi_delay=epsi - v * delta / 2.67 * delay_t;

		  Eigen::VectorXd state(6);
		  state << x_delay, y_delay, psi_delay, v_delay, cte_delay, epsi_delay;

		  //
		  double start, stop;
		  
		  start = clock();
		  count++;
		  vector<double> result = mpc.Solve(state, coeffs);
		  stop = clock();
		  durationTime += ((double)(stop - start)) / CLOCKS_PER_SEC;
		  if (count == 10){
		      std::cout << "Time consume" << durationTime << std::endl;
		  }
		  

          double steer_value= result[0];
		  //std::cout << "steervalue " << steer_value << std::endl;
          double throttle_value=result[1];
		  //std::cout << "throttle_value " << throttle_value << std::endl;

          json msgJson;
          // NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
          // Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].
          msgJson["steering_angle"] = steer_value/ deg2rad(25);
          msgJson["throttle"] = throttle_value;

          //Display the MPC predicted trajectory 
		  vector<double> mpc_x_vals;
          vector<double> mpc_y_vals;
		  for (int i = 2; i <= N-1; i++) {
			  mpc_x_vals.push_back(result[i]);
		  }

		  for (int i = N; i <= 2*N-3; i++) {
			  mpc_y_vals.push_back(result[i]);
		  }

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Green line

          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;

          //Display the waypoints/reference line
          vector<double> next_x_vals;
          vector<double> next_y_vals;
		 
		  /*for (int i = 0; i < local_ptsx.size(); i++) {
			  next_x_vals.push_back(local_ptsx[i]);
		  }
		  for (int i = 0; i < local_ptsy.size(); i++) {
			  next_y_vals.push_back(local_ptsy[i]);
		  }*/
		  for (double i = 0; i < 100; i += 3) {
			  next_x_vals.push_back(i);
			  next_y_vals.push_back(polyeval(coeffs, i));
		  }
		  
          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Yellow line

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          //std::cout << msg << std::endl;

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
