#include <stdexcept>
#include <iostream>
#include "brickpi/PidController.hpp"

namespace brickpi {

double PidController::computeOutput(double error, uint64_t time_nanosec,
                                    bool & isSaturated)
{
  // check timestamp
  if (time_nanosec < lastTime_) {
    throw std::runtime_error("Timestamps must be monotonically increasing!");
  }

  // compute time difference since last time:
  double dt = 0.0;
  if (lastTime_ != 0) {
    dt = (double(time_nanosec - lastTime_) * 1.0e-9);  // [sec]
  }
  if (dt>0.03) {
    std::cout<<"warning: dt = "<<dt<< " > 0.03"<<std::endl;
  }

  // compute the error derivative:
  double d_error_dt = 0.0;
  if (dt > 1.0e-12) {
    d_error_dt = (error - lastError_) / dt;
  }

  // compute output:
  double output = parameters_.k_p * error + parameters_.k_i * integratedError_
      + parameters_.k_d * d_error_dt;

  // handle saturation:
  if (output > parameters_.maxOutput) {
    output = parameters_.maxOutput;  // saturate output - no error integration
    isSaturated = true;
  } else if (output < parameters_.minOutput) {
    output = parameters_.minOutput;  // saturate output - no error integration
    isSaturated = true;
  } else {
    integratedError_ += dt * error;  // in this case, we can do the error integration
    isSaturated = false;
  }

  // shift memory:
  lastTime_ = time_nanosec;
  lastError_ = error;

  return output;
}

double PidController::computeOutput(double error, uint64_t time_nanosec)
{
  bool isSaturated;
  double output = computeOutput(error, time_nanosec, isSaturated);
  if (isSaturated) {
    std::cout << "PID control signal saturated!" << std::endl;
  }
  return output;
}

}  // namespace brickpi
