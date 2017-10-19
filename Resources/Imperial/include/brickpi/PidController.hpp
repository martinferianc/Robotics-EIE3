/* written by sleutenegger, 18th December 2014 */

#ifndef INCLUDE_BRICKPI_PIDCONTROLLER_HPP_
#define INCLUDE_BRICKPI_PIDCONTROLLER_HPP_

#include <stdint.h>

namespace brickpi {

/**
 * Generic PID controller class with anti-reset-windup.
 */
class PidController
{
 public:
  /**
   * Helper parameter struct.
   */
  struct Parameters
  {
    Parameters()
        : minOutput(0.0),
          maxOutput(0.0),
          k_p(0.0),
          k_i(0.0),
          k_d(0.0)
    {
    }
    Parameters(double minOutput, double maxOutput, double k_p, double k_i,
               double k_d)
        : minOutput(minOutput),
          maxOutput(maxOutput),
          k_p(k_p),
          k_i(k_i),
          k_d(k_d)
    {
    }
    double minOutput;  ///< min saturation
    double maxOutput;  ///< max saturation
    double k_p;  ///< proportional gain
    double k_i;  ///< integral gain
    double k_d;  ///< derivative gain
  };

  PidController()
      : lastTime_(0),
        lastError_(0.0),
        integratedError_(0.0)
  {
  }
  PidController(const Parameters & parameters)
      : parameters_(parameters),
        lastTime_(0),
        lastError_(0.0),
        integratedError_(0.0)
  {
  }

  void setParamteters(const Parameters & parameters)
  {
    parameters_ = parameters;
  }

  void getParameters(Parameters & parameters) const
  {
    parameters = parameters_;
  }

  /// computing the control output: version that prints a warning on saturation
  double computeOutput(double error, uint64_t time_nanosec);
  /// computing the control output: version let's you know about saturation
  double computeOutput(double error, uint64_t time_nanosec, bool & isSaturated);

  /// guarantee smooth change between manual and automatic:
  void reset()
  {
    lastTime_=0;
    integratedError_=0.0;
  }

 private:
  Parameters parameters_;  ///< store the parameters
  uint64_t lastTime_;  ///< remember when computing the last output
  double lastError_;  ///< remember what was the error when computing the last output
  double integratedError_;
};

}  // namespace brickpi

#endif /* INCLUDE_BRICKPI_PIDCONTROLLER_HPP_ */ 
