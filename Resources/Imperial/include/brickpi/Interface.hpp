/* written by sleutenegger, 17th December 2014 */

#ifndef INCLUDE_BRICKPI_INTERFACE_HPP_
#define INCLUDE_BRICKPI_INTERFACE_HPP_

#include <stdint.h>
#include <vector>
#include <boost/thread/thread.hpp>
#include <atomic>
#include <boost/scoped_ptr.hpp>
#include "brickpi/PidController.hpp"
#include <boost/timer/timer.hpp>
#include <boost/python/tuple.hpp>
#include <boost/python/list.hpp>
#include <boost/python/extract.hpp>
#include <fstream>

namespace brickpi {

/**
 * BrickPi Interface class.
 * We mirror the functionalities provided by the manufacturer in C++ style
 * and additionally provide (somewhat) low-level speed and position control for the motors.
 */
class Interface
{
 public:
  Interface();
  ~Interface();

  Interface(const Interface&) = delete;

  static const double TICKS_PER_RAD;  // 720.0 / (2.0 * M_PI);
  static const double CONTROL_INTERVAL_SECONDS;  // 0.01, i.e. 100 Hz
  static const int ULTRASONIC_INTERVAL_MILLISECONDS;

  typedef uint64_t timestamp_t;
  boost::timer::cpu_timer timer;

  template<typename T>
  void python_list_to_std_vector(const boost::python::list &l, std::vector<T> &v)
  {
    v.resize(boost::python::len(l));
    for(int i=0; i<v.size(); i++)
    {
      v[i] = boost::python::extract<T>(l[i]);
    }
  }

  /// this starts a separate thread that continuously polls the activated sensors
  /// as well as controls the motors that were started:
  bool initialize();
  /// softly stop all motors and sensors, stop the polling and control thread:
  bool terminate();
  /// immediately stop all motors, stop the polling and control thread (bad for the hardware):
  bool emergencyStop();

  /// sensor types are public so they can be accessed by the user via Python
  enum SensorType
  {
    SENSOR_ULTRASONIC,
    SENSOR_TOUCH,
    SENSOR_NONE,
  };

  /// start individual motors:
  bool motorEnable(unsigned char motorPort);
  /// stop individual motors:
  bool motorDisable(unsigned char motorPort);
  /// activate individual sensors:
  bool sensorEnable(unsigned char sensorPort, SensorType sensorType);
  /// deactivate individual sensors:
  bool sensorDisable(unsigned char sensorPort);

  /// thread safe access to sensor values - returns false in case of an error
  bool getSensorValue(unsigned char sensorPort, long &value);
  /// thread safe access to sensor values - returns false in case of an error
  bool getSensorValue(unsigned char sensorPort, long &value,
                      timestamp_t &timestamp);

  boost::python::tuple getSensorValuePython(unsigned char sensorPort);

  /// low-level motor interface -- overrides controller:
  bool setMotorPwm(unsigned char motorPort, int pwm);

  /// low-level motor speed controller interface:
  struct MotorAngleControllerParameters
  {
    MotorAngleControllerParameters()
        : maxRotationAcceleration(0.0),
          maxRotationSpeed(0.0),
          feedForwardGain(0.0),
          minPWM(0.0)
    {  // TODO sleutenegger: find default values
    }
    MotorAngleControllerParameters(
        double maxRotationAcceleration, double maxRotationSpeed,
        double feedForwardGain, double minPwm, const PidController::Parameters & pidParameters)
        : maxRotationAcceleration(maxRotationAcceleration),
          maxRotationSpeed(maxRotationSpeed),
          pidParameters(pidParameters),
          feedForwardGain(feedForwardGain),
          minPWM(minPwm)
    {
    }
    double maxRotationAcceleration;  ///< maximum admissible rotation acceleration [rad/s^2]
    double maxRotationSpeed;  ///< maximum admissible rotation speed [rad/s]
    double feedForwardGain;  ///< gain on operating on the desired rotation speed as feed-forward
    double minPWM;
    PidController::Parameters pidParameters;  ///< PID controller parameters
  };

  /// set the controller parameters to non-default values:
  bool setMotorAngleControllerParameters(
      unsigned char motorPort,
      const MotorAngleControllerParameters & motorAngleControllerParameters);

  /*
   * Speed Controller (rotation speed) --
   * this internally also uses the position controller (and its parameters)
   */
  /// set a controller speed reference -- overrides low-level setMotorPwm:
  bool setMotorRotationSpeedReference(unsigned char motorPort,
                                      double rotationSpeedReference);
  /// set controller speed references -- overrides low-level setMotorPwm.
  /// this version guarantees synchronous operation
  bool setMotorRotationSpeedReferences(
      const std::vector<unsigned char> & motorPorts,
      const std::vector<double> & rotationSpeedReferences);
  //Python alternative
  bool setMotorRotationSpeedReferencesPython(
      const boost::python::list & motorPorts,
      const boost::python::list & rotationSpeedReferences);
  /// query the current setpoint:
  bool getMotorRotationSpeedReference(unsigned char motorPort,
                                      double & rotationSpeedReference);
  /// figure out, if the set speed reference has been reached
  bool motorRotationSpeedReferenceReached(unsigned char motorPort);

  /*
   * Position Controller (angle)
   */
  /// set a controller angle reference -- overrides low-level setMotorPwm:
  bool setMotorAngleReference(unsigned char motorPort, double angleReference);
  /// set a controller angle reference -- overrides low-level setMotorPwm:
  bool setMotorAngleReference(unsigned char motorPort, double angleReference,
                              double rotationSpeedReference);
  /// set controller speed references -- overrides low-level setMotorPwm.
  /// this version guarantees synchronous
  bool setMotorAngleReferences(const std::vector<unsigned char> & motorPorts,
                               const std::vector<double> & angleReferences);
  //Python list version of the above
  bool setMotorAngleReferencesPython(
      const boost::python::list & motorPorts,
      const boost::python::list & angleReferences);
  /// set controller speed references -- overrides low-level setMotorPwm.
  /// this version guarantees synchronous
  bool setMotorAngleReferences(
      const std::vector<unsigned char> & motorPorts,
      const std::vector<double> & angleReferences,
      const std::vector<double> & rotationSpeedReferences);
  bool setMotorAngleReferencesPython(
      const boost::python::list & motorPorts,
      const boost::python::list & angleReferences,
      const boost::python::list & rotationSpeedReferences);
  /// increase a controller angle reference -- overrides low-level setMotorPwm:
  bool increaseMotorAngleReference(unsigned char motorPort,
                                   double angleReferenceIncrease);
  /// increase a controller angle reference -- overrides low-level setMotorPwm:
  bool increaseMotorAngleReference(unsigned char motorPort,
                                   double angleReferenceIncrease,
                                   double rotationSpeedReference);
  /// increase controller speed references -- overrides low-level setMotorPwm.
  /// this version guarantees synchronous
  bool increaseMotorAngleReferences(
      const std::vector<unsigned char> & motorPorts,
      const std::vector<double> & angleReferenceIncreases);
  bool increaseMotorAngleReferencesPython(
      const boost::python::list & motorPorts,
      const boost::python::list & angleReferenceIncreases);
  /// increase controller speed references -- overrides low-level setMotorPwm.
  /// this version guarantees synchronous
  bool increaseMotorAngleReferences(
      const std::vector<unsigned char> & motorPorts,
      const std::vector<double> & angleReferenceIncreases,
      const std::vector<double> & rotationSpeedReferences);
  // Python alternative
  bool increaseMotorAngleReferencesPython(
      const boost::python::list & motorPorts,
      const boost::python::list & angleReferenceIncreases,
      const boost::python::list & rotationSpeedReferences);
  /// query the current setpoint:
  bool getMotorAngleReference(unsigned char motorPort, double & angleReference);
  boost::python::list getMotorAngleReferencesPython(const boost::python::list & motorPorts);
  /// query the current (actual) motor speed
  bool getMotorAngle(unsigned char motorPort, double & angle,
                     timestamp_t & timestamp);
  boost::python::tuple getMotorAnglePython(unsigned char motorPort);
  /// query the current (actual) motor speed
  bool getMotorAngle(unsigned char motorPort, double & angle);
  /// query the current (actual) motor speeds
  bool getMotorAngles(const std::vector<unsigned char>& motorPorts,
                      std::vector<double> & angles,
                      std::vector<timestamp_t> & timestamps);
  boost::python::list getMotorAnglesPython(const boost::python::list & motorPorts);
  /// query the current (actual) motor speeds
  bool getMotorAngles(const std::vector<unsigned char>& motorPorts,
                      std::vector<double> & angles);
  /// figure out, if the set angle reference has been reached
  bool motorAngleReferenceReached(unsigned char motorPort);
  bool motorAngleReferencesReachedPython(const boost::python::list & motorPorts);

  // TODO sleutenegger / janjachnik: an interface for the rest of the sensors...

  bool startLogging(std::string fileName);
  bool stopLogging();

 private:
  /// this is the background thread handling all sensor polling and motor output setting
  int controlAndSensingLoop();
  /// this fucntion defines our feed-forward gain model
  double deadbandCompensatedPwm(unsigned char motorPort, double pwm);
  /// this controls all motors
  bool controlMotors(timestamp_t timestamp);
  /// this calls brickPiUpdateValues
  bool updateValues(bool getUltrasonic, timestamp_t & timestamp);
  /// this calls brickPiSetupSensors
  bool setupSensors();

  /// two ways of filtering the references, depending on mode:
  bool filterVelocityMode(double dt, unsigned char motorPort);
  /// two ways of filtering the references, depending on mode:
  bool filterPositionMode(double dt, unsigned char motorPort);

  MotorAngleControllerParameters motorAngleControllerParameters_[4];  ///< the stored controller parameters
  std::atomic<double> pwmReferences_[4];  ///< these may be set by the user
  std::atomic<double> rotationSpeedReferences_[4];  ///< these may be set by the user
  std::atomic<double> angleReferences_[4];  ///< these may be set by the user
  double motorAngles_[4];  ///< actual angle -- can be queried by the user
  std::atomic<double> filteredMaxRotationAccelerations_[4];  ///< these are constantly calculated by respecting limits
  std::atomic<double> filteredMaxRotationDecelerations_[4];  ///< these are constantly calculated by respecting limits
  std::atomic<bool> inDeceleration_[4]; ///< indicates, if deceleration has started
  std::atomic<bool> referenceReached_[4]; ///< indicates, if set velocity or position reference was reached
  std::atomic<double> filteredRotationSpeedReferences_[4];  ///< these are constantly calculated by respecting limits
  std::atomic<double> filteredAngleReferences_[4];  ///< these are constantly calculated by respecting limits
  std::atomic<timestamp_t> motorTimestamp;

  struct sensorValue_t
  {
    long value;
    timestamp_t timestamp;
    sensorValue_t(long _val, timestamp_t _ts)
        : value(_val),
          timestamp(_ts)
    {
    }
    sensorValue_t()
        : value(0),
          timestamp(0)
    {
    }
  };

  std::atomic<SensorType> sensorTypes_[4];
  sensorValue_t sensorValues_[4];  ///< Sensor values to be queried by user
  std::atomic<bool> ultrasonicEnabled;

  /// copies data into internal structs
  bool updateSensorValues(bool getUltrasonic, timestamp_t timestamp);

  // controllers
  PidController pidControllers_[4];

  unsigned long int lastTime_;

  enum ControlMode
  {
    Disabled,
    Manual,
    Velocity,
    Position
  };
  std::atomic<ControlMode> controlModes_[4];

  // multithreading:
  std::atomic<bool> shutdown_;
  boost::scoped_ptr<boost::thread> controlAndSensingThread_;
  boost::mutex mutex_[4];  ///< protect access of motorAngleControllerParameters_
  boost::mutex commMutex_;  ///< protect writing to UART
  boost::mutex sensMutex_;  ///< protect writing to sensor values

  //logging
  std::ofstream logFile;
  std::atomic<bool> loggingEnabled;
};

}  // namespace brickpi

#endif /* INCLUDE_BRICKPI_INTERFACE_ */
