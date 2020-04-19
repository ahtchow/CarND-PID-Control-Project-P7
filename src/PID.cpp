#include "PID.h"

/**
 * TODO: Complete the PID class. You may add any additional desired functions.
 */

PID::PID() {}

PID::~PID() {}


void PID::Init(double Kp_, double Ki_, double Kd_) {

  Kp = Kp_;
  Ki = Ki_;
  Kd = Kd_;
  p_error = 0;
  i_error = 0;
  d_error = 0;
  prev_CTE = 0;
  have_i_twiddled = false;

}

void PID::UpdateError(const double CTE) {

  p_error = CTE;
  i_error += CTE;
  d_error = CTE-prev_CTE;
  prev_CTE = CTE;

}

double PID::TotalError(){

  return -Kp * p_error - Ki * i_error - Kd * d_error;

}

void PID::ExecuteUpdate(
    const double steer_value,
    const double throttle,
    uWS::WebSocket<uWS::SERVER> ws){
  
  nlohmann::json msgJson;
  msgJson["steering_angle"] = steer_value;
  msgJson["throttle"] = throttle;
  auto msg = "42[\"steer\"," + msgJson.dump() + "]";
  // std::cout << msg << std::endl;
  ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

}

bool PID::Twiddle(const vector<double> INCREMENT, const int MAX_STEPS){

  vector<double> curr_K;
  curr_K.push_back(Kp);
  curr_K.push_back(Ki);
  curr_K.push_back(Kd);

  double best_error = TotalError();
  double curr_error = std::numeric_limits<double>::max();
  vector<double> K_increment = INCREMENT;
  size_t steps = 0;

  if(!have_i_twiddled){
    have_i_twiddled = true;
    return false;
  }

  while(steps < MAX_STEPS){

    for(size_t i = 0; i < K_increment.size(); ++i){
      
      vector<double> orig_K_increment = K_increment;
      //Calculate New Error
      curr_K[i] += K_increment[i];
      curr_error = -curr_K[0] * p_error - curr_K[1] * i_error - curr_K[2] * d_error;
      
      if(fabs(curr_error) < fabs(best_error)){
        
        //Increase Increment if Successful
        best_error = curr_error;
        K_increment[i] *= 1.1;
      
      }

      else{

        //Decrease by one increment, test new error
        curr_K[i] -= 2.0*K_increment[i];
        curr_error = -curr_K[0] * p_error - curr_K[1] * i_error - curr_K[2] * d_error;
      
        if(fabs(curr_error) < fabs(best_error)){

          //Increase Increment if Successful
          best_error = curr_error;
          K_increment[i] *= 1.1;

        }

        else{

          //Return Original
          curr_K[i] += K_increment[i];
          K_increment[i] = orig_K_increment[i];

        }

      }

    }
    ++steps;
  }

  // No changes
  if(curr_K[0] == Kp && curr_K[1] == Ki && curr_K[2] == Kd)
    return false;

  Kp = curr_K[0];
  Ki = curr_K[1];
  Kd = curr_K[2];
  return true;

}
