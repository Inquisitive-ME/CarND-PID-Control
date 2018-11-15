#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include <math.h>

void addToBuffer(double cte, double (&buffer)[5])
{
  for(int i = 0; i < 4; i++)
  {
    buffer[i+1] = buffer[i];
  }
  buffer[0] = cte;
}

double average(double buffer[5], double weights[5])
{
  double total = 0;
  for(int i = 0; i < 5; i++)
  {
    //std::cout << "buffer[i] = " << buffer[i] << "weights[i] = " << weights[i] << std::endl;
    total += (buffer[i] * weights[i]);
  }
  return (total);
}

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

int main()
{
  uWS::Hub h;

  PID pid_speed;
  PID pid_steering;
  pid_speed.Init(.15,0.0008,0.3);
  pid_steering.Init(.5,.0003,2);
  // TODO: Initialize the pid variable.
  double sumSteering = 0;
  int i = 0;
  bool firstTime = true;
  double cteBuffer[5];
  for(int i = 0; i < 5; i++)
  {
    cteBuffer[i] = 0;
  }

  h.onMessage([&pid_steering, &pid_speed, &sumSteering, &i, &firstTime, &cteBuffer](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
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
          double throttle_value;
          double filterCTE;

          pid_speed.UpdateError(speed - 20);
          throttle_value = pid_speed.TotalError();
         // std::cout << "speed p_error = " << pid_speed.p_error << " i_error = " << pid_speed.i_error << " d_error = " << pid_speed.d_error <<
         // " throttle = " << throttle_value << std::endl;


          /*
          * TODO: Calcuate steering value here, remember the steering value is
          * [-1, 1].
          * NOTE: Feel free to play around with the throttle and speed. Maybe use
          * another PID controller to control the speed!
          */
          if(i == 0)
          {
            for(int j = 0; j < 5; j++)
            {
              cteBuffer[j] = cte;
            }
          }
          else
          {
            addToBuffer(cte, cteBuffer);
          }
          i++;
          if(i>400){
            std::cout << "Average cte = " << sumSteering/i << std::endl;
            if(firstTime)
            {
              // TODO move this into the twiddle function
              pid_steering.setBestError(sumSteering/i);
              firstTime = false;
            }
            pid_steering.UpdateAndTwiddle(sumSteering/i,0.01);
            i=0;
            sumSteering = 0;
            steer_value = 0;
            throttle_value = 0;
            for(int i = 0; i < 5; i++)
            {
              cteBuffer[i] = 0;
            }
            std::string reset_msg = "42[\"reset\",{}]";
            ws.send(reset_msg.data(), reset_msg.length(), uWS::OpCode::TEXT);
          }
          else
          {
            sumSteering += fabs(cte);
            double weights[5];
            weights[0] = 0.8;
            weights[1] = 0.1;
            weights[2] = 0.05;
            weights[3] = 0.05;
            weights[4] = 0.0;
            filterCTE = average(cteBuffer, weights);
            std::cout << "CTE = " << cte << "Filtered = " << filterCTE << std::endl;
            pid_steering.UpdateError(filterCTE);
            steer_value = pid_steering.TotalError();
          }
          
          // DEBUG
          //std::cout << "CTE: " << cte << " Steering Value: " << steer_value << std::endl;

          //Filter Output Steering
          //steer_value = ((exp(steer_value)/(1 + exp(steer_value))) - 0.5) * 2;

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          //std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
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
