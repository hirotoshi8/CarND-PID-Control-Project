#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include <math.h>
#include <vector>

using namespace std;

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

enum DIRECTION{UP, DOWN};
DIRECTION direction;

class Twiddle {
public:
	Twiddle(int max_iteration, double d_Kp, double d_Ki, double d_Kd, double eps);
	//Twiddle(int max_iteration, double d_Kp, double d_Ki, double d_Kd, double eps);
	/* twiddle time is over ? */
	int is_over_twiddle_time(void);
	void add_step(void);

	int is_enough_twiddle(void);

	/* Update control gain */
	void update_gain_up(PID *pid); // Upper side
	void update_gain_dw(PID *pid); // Lower side
	
	/* Change the target gain (Kp, Ki Kd)*/
	void change_target_gain(void);

	/* Return the result of the control gain */
	int is_error_decreased(double cte);

	/* Update the delta gain*/
	void update_delta_gain(double delta);
	double best_error;

	double delta_Kp;
	double delta_Ki;
	double delta_Kd;


private:
	int twiddle_steps;
	int max_twiddle_steps;
	double eps_;
	//double delta_Kp;
	//double delta_Ki;
	//double delta_Kd;
	enum TARGET_GAIN { TARGET_P, TARGET_I, TARGET_D };
	TARGET_GAIN target_gain;
};

Twiddle::Twiddle(int max_iteration, double d_Kp, double d_Ki, double d_Kd, double eps=0.01) {
	this->max_twiddle_steps = max_iteration;
	this->delta_Kp = d_Kp;
	this->delta_Ki = d_Ki;
	this->delta_Kd = d_Kd;
	this->target_gain = TARGET_P;
	this->best_error = 1;
	this->eps_ = eps;
}

void Twiddle::change_target_gain(void) {
	/* Change the target gain (Kp, Ki Kd)*/
	if      (this->target_gain == TARGET_P) { this->target_gain = TARGET_I; }
	else if (this->target_gain == TARGET_I) { this->target_gain = TARGET_D; }
	else if (this->target_gain == TARGET_D) { this->target_gain = TARGET_P; }
	else {
		cout << "---------- Error:update_target ---------- " << endl;
	}
	return;
}

void Twiddle::update_gain_up(PID *pid) {
	/* Increse the control gain */
	if      (this->target_gain == TARGET_P){ pid->Kp_ += this->delta_Kp;}
	else if (this->target_gain == TARGET_I){ pid->Ki_ += this->delta_Ki;}
	else if (this->target_gain == TARGET_D){ pid->Kd_ += this->delta_Kd;}
	else {
		cout << "Error:update_gain_up " << endl;
	}
	return;
}

void Twiddle::update_gain_dw(PID *pid) {
	/* Decrese the control gain */
	if      (this->target_gain == TARGET_P) { pid->Kp_ -= 2*this->delta_Kp;}
	else if (this->target_gain == TARGET_I) { pid->Ki_ -= 2*this->delta_Ki;}
	else if (this->target_gain == TARGET_D) { pid->Kd_ -= 2*this->delta_Kd;}
	else {
		cout << "Error:update_gain_dw " << endl;
	}
	return;
}

int Twiddle::is_error_decreased(double cte) {
	/* Return the result of the control gain */

	int error_ret = false;

	/* Update best error */
	if (abs(this->best_error) >= abs(cte)) {
		this->best_error = cte;

		error_ret = true;
	}
	else {
		/* Nothing to do */
	}
	return error_ret;
}

void Twiddle::update_delta_gain(double delta) {
	/* Update delta gains */
	if      (this->target_gain == TARGET_P) { this->delta_Kp *= delta;}
	else if (this->target_gain == TARGET_I) { this->delta_Ki *= delta;}
	else if (this->target_gain == TARGET_D) { this->delta_Kd *= delta;}
	else{ cout << "Error: update_delta_gain " << endl;}

	return;
}

void Twiddle::add_step(void) {
	this->twiddle_steps++;
}

int Twiddle::is_enough_twiddle(void) {

	int is_enough = false;

	double sum;
	sum = this->delta_Kp + this->delta_Ki + this->delta_Kd;

	if (abs(eps_) > abs(sum)) {
		is_enough = true;
	}
	return is_enough;
}

int Twiddle::is_over_twiddle_time(void) {
	
	int is_over = false;

	if (twiddle_steps > this->max_twiddle_steps) {
		is_over = true;
		return is_over;
	}
	return is_over;
}

static Twiddle twiddle(500, 0.01, 0.001, 1, 0.001);//200
//Twiddle::Twiddle(int max_iteration, double d_Kp, double d_Ki, double d_Kd, double eps = 0.01)

int main()
{
  uWS::Hub h;

  PID pid;
  // TODO: Initialize the pid variable.
  double Kp = 0.1;
  double Ki = 0.01;
  double Kd = 20;
  pid.Init(Kp, Ki, Kd);

  h.onMessage([&pid](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
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
			/*
			* TODO: Calcuate steering value here, remember the steering value is
			* [-1, 1].
			* NOTE: Feel free to play around with the throttle and speed. Maybe use
			* another PID controller to control the speed!
			*/

			static int state = 0;
			static int is_end_twiddle = false;

			if ( (!twiddle.is_over_twiddle_time() | !twiddle.is_enough_twiddle()) & !is_end_twiddle )
			{
				//debug
				cout << "---------- Twiddle time ! ---------- " << endl;
				cout << "---------- State: " << state << endl;
				cout << "---------- Kp: " << pid.Kp_<< endl;
				cout << "---------- Ki: " << pid.Ki_ << endl;
				cout << "---------- Kd: " << pid.Kd_ << endl;
				cout << "---------- delta_Kp: " << twiddle.delta_Kp << endl;
				cout << "---------- delta_Ki: " << twiddle.delta_Ki << endl;
				cout << "---------- delta_Kd: " << twiddle.delta_Kd << endl;


				/* Update Control Gain Upper direction */
				if (state == 0) {
					twiddle.update_gain_up(&pid);
					state = 1;
					//debug
					cout << "---------- State: " << 00 << endl;
					cout << "---------- Error: " << cte << endl;
				}
				else if(state == 1){
					/* Check the result of Upper direction update */
					if (twiddle.is_error_decreased(cte)) {
						/* Success */
						/* Adjust the delta gain */
						twiddle.update_delta_gain(1.1);
						/* Change the target gain */
						twiddle.change_target_gain();
						state = 0;
						//debug
						cout << "---------- State: " << 10 << endl;
						cout << "---------- Error: " << cte << endl;
					}
					else {
						/* Fail */
						/* Update Control Gain Lower direction */
						twiddle.update_gain_dw(&pid);
						state = 2;
						//debug
						cout << "---------- State: " << 11 << endl;
						cout << "---------- Error: " << cte << endl;
					}
				}
				else if (state == 2) {
					/* Check the result of Lower direction update */
					if (twiddle.is_error_decreased(cte)) {
						/* Success */
						/* Adjust the delta gain as same as before */
						twiddle.update_delta_gain(1.0);
						/* Change the target gain */
						twiddle.change_target_gain();
						state = 0;
						//debug
						cout << "---------- State: " << 20 << endl;
						cout << "---------- Error: " << cte << endl;
					}
					else {
						/* Fail */
						/* Update(Reverse) the control gain */
						twiddle.update_gain_up(&pid);
						/* Adjust the delta gain ( Smaller step size ) */
						twiddle.update_delta_gain(0.9);
						/* Change the target gain */
						twiddle.change_target_gain();
						state = 0;
						//debug
						cout << "---------- State: " << 21 << endl;
						cout << "---------- Error: " << cte << endl;
					}

				}
				else {
					//debug
					cout << "---------- Error State move ! ---------- " << endl;
				}
				twiddle.add_step();
			}
			else {
				/* Nothig to do */
				is_end_twiddle = true;
			}

		  /* Set the steer_value */
			pid.UpdateError(cte);
			steer_value = pid.TotalError();
			

			static double pre_steer_value = 0;
			if ( pre_steer_value*steer_value < 0) { pid.i_error_ = 0; }
			pre_steer_value = steer_value;	

			/* Limiter for steering */
			if (steer_value > 1) {
				steer_value = 1;
			}else if(steer_value < -1){
				steer_value = -1;
			}
			else {
				/* Nothing to do */
			}

          // DEBUG
          std::cout << "CTE: " << cte << " Steering Value: " << steer_value << std::endl;
		  std::cout << "Kp: " << pid.Kp_ << " Ki: " << pid.Ki_ << " Kd: " << pid.Kd_ << std::endl << endl;

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = 0.3;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
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
