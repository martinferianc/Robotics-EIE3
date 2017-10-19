/* written by sleutenegger 18th Dec 2014 */

#include <iostream>
#include <iomanip>

#include <boost/thread/thread.hpp>
#include <boost/timer/timer.hpp>

#include "BrickPi.h"

#include "brickpi/Interface.hpp"

namespace brickpi {

void checkValidPort(unsigned char port)
{
  if (port > 3) {
    throw std::runtime_error("invalid port requested.");
  }
}

const double Interface::TICKS_PER_RAD = 720.0 / (2.0 * M_PI);
const double Interface::CONTROL_INTERVAL_SECONDS = 0.01;  // 100 Hz
const int Interface::ULTRASONIC_INTERVAL_MILLISECONDS = 15;

Interface::Interface()
{
  for (size_t i = 0; i < 4; ++i) {
    // set all references to zero, manual control
    pwmReferences_[i].store(0.0);  ///< these may be set by the user
    rotationSpeedReferences_[i].store(0.0);  ///< these may be set by the user
    angleReferences_[i].store(0.0);  ///< these may be set by the user
    motorAngles_[i] = (0.0);  ///< these may be queried by the user
    filteredMaxRotationAccelerations_[i].store(0.0);  ///< these are constantly calculated by respecting limits
    filteredRotationSpeedReferences_[i].store(0.0);  ///< these are constantly calculated by respecting limits
    filteredAngleReferences_[i].store(0.0);  ///< these are constantly calculated by respecting limits
    inDeceleration_[i] = false;
    controlModes_[i].store(Disabled);
    sensorTypes_[i] = SENSOR_NONE;
    referenceReached_[i] = false;
  }
  loggingEnabled = false;
  timer.start();

  ultrasonicEnabled.store(false);

  shutdown_.store(false);  // no shutdown

  lastTime_ = 0;

  // reset the clock
  ClearTick();
}

Interface::~Interface()
{
  // check if control loop is already running
  bool success = true;
  if (!controlAndSensingThread_) {
    for (size_t i = 0; i < 4; ++i) {
      if (controlModes_[i].load() == Disabled) {
        continue;  // we will ignore the disable ones
      }
      success &= setMotorPwm(i, 0);
    }
  } else {
    terminate();
  }
  std::cout.flush();
}

/// this starts a separate thread that continuously polls the activated sensors
/// as well as controls the motors that were started:
bool Interface::initialize()
{

  // make sure this is only called once.
  if (controlAndSensingThread_) {
    std::cout << "initialized called before, ignoring." << std::endl;
    return false;
  }

  int result = BrickPiSetup();
  printf("BrickPiSetup: %d\n", result);
  if (result != 0) {
    return false;
  }

  // TODO sleutene: figure out what these two really do...
  BrickPi.Address[0] = 1;
  BrickPi.Address[1] = 2;

  // start the sensing and control loop
  controlAndSensingThread_.reset(
      new boost::thread(&Interface::controlAndSensingLoop, this));

  return true;
}

/// softly stop all motors and sensors, stop the polling and control thread:
bool Interface::terminate()
{
  std::cout << "terminate..." << std::endl;
  // check if control loop is already running
  bool success = true;
  if (!controlAndSensingThread_) {
    for (size_t i = 0; i < 4; ++i) {
      if (controlModes_[i].load() == Disabled) {
        continue;  // we will ignore the disable ones
      }
      success &= setMotorPwm(i, 0);
    }
    return success;
  }

  // get current velocities and estimate shutdown time
  double shutdownTime = 0.0;
  for (size_t i = 0; i < 4; ++i) {
    if (controlModes_[i].load() == Disabled) {
      continue;  // we will ignore the disable ones
    }
    double speed = rotationSpeedReferences_[i];
    double acc = motorAngleControllerParameters_[i].maxRotationAcceleration;
    if (acc < 0.1) {
      acc = 0.1;
    }
    shutdownTime = std::max(speed / acc, shutdownTime);
  }

  // shutdown all motors synchronously
  std::vector<unsigned char> motorPorts;
  std::vector<double> rotationSpeedReferences;
  for (size_t i = 0; i < 4; ++i) {
    if (controlModes_[i].load() == Disabled || Manual) {
      continue;  // we will ignore the disable ones
    }
    motorPorts.push_back(i);
    rotationSpeedReferences.push_back(0.0);
  }
  success = setMotorRotationSpeedReferences(motorPorts,
                                            rotationSpeedReferences);

  // wait for stop completion - maximum twice the calculated time
  bool running = true;
  boost::timer::cpu_timer timer;
  boost::timer::nanosecond_type time_delay(2.0 * shutdownTime * 1.0e9);
  timer.start();
  while (running) {
    running = false;
    for (size_t i = 0; i < 4; ++i) {
      if (controlModes_[i].load() == Disabled || Manual) {
        continue;  // we will ignore the disabled/manual ones
      }
      if (fabs(filteredRotationSpeedReferences_[i].load()) > 1.0e-9) {
        running = true;
      }
    }
    if (timer.elapsed().wall > time_delay && running) {
      std::cout << "shutdown taking longer than expected, forcing..."
                << std::endl;
      break;
    }
  }

  // kill controlAndSensingLoop
  shutdown_.store(true);
  controlAndSensingThread_->join();
  controlAndSensingThread_.reset(NULL);

  // send PWM 0
  for (size_t i = 0; i < 4; ++i) {
    success &= setMotorPwm(i, 0);
  }
  return success;
}

/// immediately stop all motors, stop the polling and control thread (bad for the hardware):
bool Interface::emergencyStop()
{
  // kill controlAndSensingLoop
  shutdown_.store(true);
  if (controlAndSensingThread_) {
    controlAndSensingThread_->join();
  }
  controlAndSensingThread_.reset(NULL);

  // send PWM 0
  bool success = true;
  for (size_t i = 0; i < 4; ++i) {
    if (controlModes_[i].load() == Disabled) {
      continue;  // we will ignore the disable ones
    }
    success &= setMotorPwm(i, 0);
  }

  std::cout << "e-stop requested, sent stop..." << std::endl;

  return success;
}

/// start and stop individual motors:
bool Interface::motorEnable(unsigned char motorPort)
{
  if (motorPort > 3) {
    return false;
  }

  if (controlModes_[motorPort].load() != Disabled) {
    std::cout << "motor " << motorPort << "was already enabled, ignoring."
              << std::endl;
    return false;
  }

  BrickPi.MotorEnable[motorPort] = 1;  //Enable the motor

  // remember state
  controlModes_[motorPort].store(Manual);

  if (!setupSensors())
    return false;

  timestamp_t timestamp;
  if (!updateValues(false, timestamp))
    return false;

  return true;
}
bool Interface::motorDisable(unsigned char motorPort)
{
  if (motorPort > 3) {
    return false;
  }

  if (controlModes_[motorPort].load() == Disabled) {
    std::cout << "motor " << motorPort << "was already disabled, ignoring."
              << std::endl;
    return false;
  }

  // send PWM 0
  bool success = true;
  success &= setMotorPwm(motorPort, 0);

  BrickPi.MotorEnable[motorPort] = 0;  //Disable the motor
  if (!setupSensors())
    return false;

  // remember state
  controlModes_[motorPort].store(Disabled);
  return true;
}
/// activate and deactivate individual sensors:
bool Interface::sensorEnable(unsigned char sensorPort, SensorType sensorType)
{
  if (sensorPort > 3) {
    return false;
  }
  switch (sensorType) {
    case SENSOR_NONE: {
      return sensorDisable(sensorPort);
    }
    case SENSOR_TOUCH: {
      BrickPi.SensorType[sensorPort] = TYPE_SENSOR_TOUCH;
      break;
    }
    case SENSOR_ULTRASONIC: {
      BrickPi.SensorType[sensorPort] = TYPE_SENSOR_ULTRASONIC_CONT;
      ultrasonicEnabled.store(true);
      break;
    }
    default: {
      return false;
    }
  }

  sensorTypes_[sensorPort].store(sensorType);

  if (!setupSensors())
    return false;

  timestamp_t timestamp;
  if (!updateValues(sensorType == SENSOR_ULTRASONIC, timestamp))
    return false;

  return true;
}
bool Interface::sensorDisable(unsigned char sensorPort)
{
  if (sensorPort > 3) {
    return false;
  }
  BrickPi.SensorType[sensorPort] = 0;

  setupSensors();

  // remember state
  sensorTypes_[sensorPort].store(SENSOR_NONE);

  //Check to see if we still have an US sensor enabled
  bool _ultrasonicEnabled = false;
  for (int i = 0; i < 4; i++) {
    if (sensorTypes_[i].load() == SENSOR_ULTRASONIC) {
      _ultrasonicEnabled = true;
    }
  }
  ultrasonicEnabled.store(_ultrasonicEnabled);

  return true;
}

/// low-level motor interface -- overrides controller:
bool Interface::setMotorPwm(unsigned char motorPort, int pwm)
{
  if (motorPort > 3) {
    return false;
  }

  // check range
  if (pwm < -255) {
    std::cout << "pwm below -255 provided, ignoring" << std::endl;
    return false;
  }
  if (pwm > 255) {
    std::cout << "pwm above 255 provided, ignoring" << std::endl;
    return false;
  }

  // put controllers to reset
  pidControllers_[motorPort].reset();

  // check, if the control loop is still running
  if (controlAndSensingThread_) {
    // in this case, we just set manual mode and reference
    controlModes_[motorPort] = Manual;
    pwmReferences_[motorPort] = pwm;
  } else {
    BrickPi.MotorSpeed[motorPort] = pwm;
    boost::lock_guard<boost::mutex> l(commMutex_);
    if (0 == BrickPiUpdateValues()) {
      return true;
    } else {
      return false;
    }
  }

  return true;
}

/// set the controller parameters to non-default values:
bool Interface::setMotorAngleControllerParameters(
    unsigned char motorPort,
    const MotorAngleControllerParameters & motorAngleControllerParameters)
{
  if (motorPort > 3) {
    return false;
  }

  boost::lock_guard<boost::mutex> l(mutex_[motorPort]);  // protect write
  // make sure the user is sensible
  if (motorAngleControllerParameters.maxRotationAcceleration < 0.0) {
    return false;
  }
  if (motorAngleControllerParameters.maxRotationSpeed < 0.0) {
    return false;
  }
  motorAngleControllerParameters_[motorPort] = motorAngleControllerParameters; 
  pidControllers_[motorPort].setParamteters(
      motorAngleControllerParameters.pidParameters);
  return true;
}

/*
 * Speed Controller (rotation speed) --
 * this internally also uses the position controller (and its parameters)
 */
/// set a controller speed reference -- overrides low-level setMotorPwm:
bool Interface::setMotorRotationSpeedReference(unsigned char motorPort,
                                               double rotationSpeedReference)
{
  if (motorPort > 3) {
    return false;
  }

  mutex_[motorPort].lock();
  // saturate
  if (rotationSpeedReference
      > motorAngleControllerParameters_[motorPort].maxRotationSpeed) {
    rotationSpeedReference = motorAngleControllerParameters_[motorPort]
        .maxRotationSpeed;
  }
  if (rotationSpeedReference
      < -motorAngleControllerParameters_[motorPort].maxRotationSpeed) {
    rotationSpeedReference = -motorAngleControllerParameters_[motorPort]
        .maxRotationSpeed;
  }

  referenceReached_[motorPort] = false;
  rotationSpeedReferences_[motorPort] = rotationSpeedReference;  // set reference

  // simply use the maximum admissible accelerations in this independent control mode

  filteredMaxRotationAccelerations_[motorPort] =
      motorAngleControllerParameters_[motorPort].maxRotationAcceleration;
  mutex_[motorPort].unlock();

  // get the current angles as a start
  if (controlModes_[motorPort] != Velocity
      && controlModes_[motorPort] != Position) {
    boost::lock_guard<boost::mutex> l(sensMutex_);
    filteredAngleReferences_[motorPort].store(motorAngles_[motorPort]);
  }

  controlModes_[motorPort] = Velocity;  // reset mode
  return true;
}

bool Interface::setMotorRotationSpeedReferencesPython(
  const boost::python::list &motorPortsList,
  const boost::python::list &rotationSpeedReferencesList)
{
  std::vector<unsigned char> motorPorts;
  std::vector<double> rotationSpeedReferences;
  python_list_to_std_vector(motorPortsList, motorPorts);
  python_list_to_std_vector(rotationSpeedReferencesList, rotationSpeedReferences);
  return setMotorRotationSpeedReferences(motorPorts, rotationSpeedReferences);
}

/// set controller speed references -- overrides low-level setMotorPwm.
/// this version guarantees synchronous operation
bool Interface::setMotorRotationSpeedReferences(
    const std::vector<unsigned char> & motorPorts,
    const std::vector<double> & rotationSpeedReferences)
{
  // sanity check
  if (motorPorts.size() != rotationSpeedReferences.size()) {
    std::cout << "unequal numbers of motor ports and references provided."
              << std::endl;
    return false;
  }

  std::vector<double> rotationSpeedReferencesCopy = rotationSpeedReferences;

  // compute synchronised acceleration/deceleration.
  // we have to figure out what takes the longest according to the set limits
  double dt = 0.0;
  for (size_t i = 0; i < motorPorts.size(); ++i) {
    const unsigned char motorPort = motorPorts[i];
    if (motorPort > 3) {
      return false;
    }

    mutex_[motorPort].lock();
    // saturate
    if (rotationSpeedReferencesCopy.at(i)
        > motorAngleControllerParameters_[motorPort].maxRotationSpeed) {
      rotationSpeedReferencesCopy.at(i) =
          motorAngleControllerParameters_[motorPort].maxRotationSpeed;
    }
    if (rotationSpeedReferencesCopy.at(i)
        < -motorAngleControllerParameters_[motorPort].maxRotationSpeed) {
      rotationSpeedReferencesCopy.at(i) =
          -motorAngleControllerParameters_[motorPort].maxRotationSpeed;
    }

    dt = std::max(
        dt,
        fabs(
            rotationSpeedReferencesCopy.at(i)
                - filteredRotationSpeedReferences_[motorPorts[i]].load())
            / motorAngleControllerParameters_[motorPorts[i]]
                .maxRotationAcceleration);
    mutex_[motorPort].unlock();
  }
  // now we save the individual adjusted limits
  boost::lock_guard<boost::mutex> l(sensMutex_);
  for (size_t i = 0; i < motorPorts.size(); ++i) {
    const unsigned char motorPort = motorPorts[i];
    double acc = (rotationSpeedReferencesCopy.at(i)
        - filteredRotationSpeedReferences_[motorPort].load()) / dt;
    filteredMaxRotationAccelerations_[motorPort] = acc;
    referenceReached_[motorPort] = false;
    rotationSpeedReferences_[motorPort] = rotationSpeedReferencesCopy.at(i);
    if (controlModes_[motorPort] != Velocity
        && controlModes_[motorPort] != Position) {
      //std::cout << "store " << motorAngles_[motorPort] << std::endl;
      filteredAngleReferences_[motorPort].store(motorAngles_[motorPort]);
    }
    controlModes_[motorPort] = Velocity;  // reset mode
  }

  return true;
}
/// query the current setpoint:
bool Interface::getMotorRotationSpeedReference(unsigned char motorPort,
                                               double & rotationSpeedReference)
{
  if (motorPort > 3) {
    return false;
  }
  rotationSpeedReference = rotationSpeedReferences_[motorPort];
  return true;
}

bool Interface::motorRotationSpeedReferenceReached(unsigned char motorPort)
{
  if (motorPort > 3) {
    return false;
  }
  if (controlModes_[motorPort] != Velocity) {
    std::cout << "motorRotationSpeedReferenceReached will only return true if in Velocity control mode." << std::endl;
    return false;
  }
  return referenceReached_[motorPort];
}

/*
 * Position Controller (angle)
 */
/// set a controller angle reference -- overrides low-level setMotorPwm:
bool Interface::setMotorAngleReference(unsigned char motorPort,
                                       double angleReference)
{
  if (motorPort > 3) {
    return false;
  }
  mutex_[motorPort].lock();
  const double maxRotationSpeed = motorAngleControllerParameters_[motorPort]
      .maxRotationSpeed;
  mutex_[motorPort].unlock();
  return setMotorAngleReference(motorPort, angleReference, maxRotationSpeed);
}
bool Interface::setMotorAngleReference(unsigned char motorPort,
                                       double angleReference,
                                       double rotationSpeedReference)
{
  if (motorPort > 3) {
    return false;
  }

  if (rotationSpeedReference < 0.0) {
    rotationSpeedReference = -rotationSpeedReference;  // ensure positive
  }

  // store angle reference
  inDeceleration_[motorPort] = false;
  referenceReached_[motorPort] = false;
  angleReferences_[motorPort] = angleReference;

  // get the needed direction...
  // first compute the time and distance needed to stop
  const double currentPosition = filteredAngleReferences_[motorPort];
  const double deltaPosition = angleReference - currentPosition;
  const double currentSpeed = filteredRotationSpeedReferences_[motorPort];
  mutex_[motorPort].lock();
  double acc = motorAngleControllerParameters_[motorPort]
      .maxRotationAcceleration;
  mutex_[motorPort].unlock();
  double dec = -acc;
  if (currentSpeed < 0.0) {
    dec = -dec;
  }
  const double dt_stop = -currentSpeed / dec;
  const double ds_stop = -0.5 * dec * dt_stop * dt_stop;

  // we use a negative slope, if
  double sign = 1.0;
  if ((ds_stop + currentPosition) > angleReference) {
    sign = -1.0;
  }
  rotationSpeedReferences_[motorPort] = sign * rotationSpeedReference;  // set speed reference

  /*std::cout << "rotationSpeedReferences_[" << int(motorPort) << "]"
   << rotationSpeedReferences_[motorPort] << std::endl;*/

  // set the acceleration and deceleration
  if (currentSpeed < rotationSpeedReferences_[motorPort]) {
    filteredMaxRotationAccelerations_[motorPort] = acc;
  } else {
    filteredMaxRotationAccelerations_[motorPort] = -acc;
  }
  filteredMaxRotationDecelerations_[motorPort] = -sign * acc;

  // get the current angles as a start
  if (controlModes_[motorPort] != Velocity
      && controlModes_[motorPort] != Position) {
    boost::lock_guard<boost::mutex> l(sensMutex_);
    filteredAngleReferences_[motorPort].store(motorAngles_[motorPort]);
    //std::cout << "filteredAngleReferences_[motorPort]" << filteredAngleReferences_[motorPort] << std::endl;
  }
  //std::cout << "filteredMaxRotationDecelerations " << filteredMaxRotationDecelerations_[motorPort] << std::endl;
  controlModes_[motorPort] = Position;  // reset mode
  return true;
}
bool Interface::setMotorAngleReferencesPython(
  const boost::python::list & motorPortsList,
  const boost::python::list & angleReferencesList)
{
  std::vector<unsigned char> motorPorts;
  python_list_to_std_vector(motorPortsList, motorPorts);
  std::vector<double> angleReferences;
  python_list_to_std_vector(angleReferencesList, angleReferences);
  return setMotorAngleReferences(motorPorts, angleReferences);
}
/// set controller speed references -- overrides low-level setMotorPwm.
/// this version guarantees synchronous
bool Interface::setMotorAngleReferences(
    const std::vector<unsigned char> & motorPorts,
    const std::vector<double> & angleReferences)
{
  // get the maxima from the parameters
  std::vector<double> rotationSpeeds(motorPorts.size());
  for (size_t i = 0; i < motorPorts.size(); ++i) {
    const unsigned char motorPort = motorPorts[i];
    if (motorPort > 3) {
      return false;
    }
    boost::lock_guard<boost::mutex> l(mutex_[motorPort]);
    rotationSpeeds[i] = motorAngleControllerParameters_[motorPort]
        .maxRotationSpeed;
  }
  return setMotorAngleReferences(motorPorts, angleReferences, rotationSpeeds);
}
bool Interface::setMotorAngleReferencesPython(
  const boost::python::list & motorPortsList,
  const boost::python::list & angleReferencesList,
  const boost::python::list & rotationSpeedReferencesList)
{
  std::vector<unsigned char> motorPorts;
  python_list_to_std_vector(motorPortsList, motorPorts);
  std::vector<double> angleReferences;
  python_list_to_std_vector(angleReferencesList, angleReferences);
  std::vector<double> rotationSpeedReferences;
  python_list_to_std_vector(rotationSpeedReferencesList, rotationSpeedReferences);
  return setMotorAngleReferences(motorPorts, angleReferences, rotationSpeedReferences);
}
bool Interface::setMotorAngleReferences(
    const std::vector<unsigned char> & motorPorts,
    const std::vector<double> & angleReferences,
    const std::vector<double> & rotationSpeedReferences)
{

  // sanity check
  if (motorPorts.size() != angleReferences.size()) {
    return false;
  }
  if (rotationSpeedReferences.size() != angleReferences.size()) {
    return false;
  }

  // check the reqested distances
  std::vector<double> deltaPositions(angleReferences.size());
  double deltaPositionsMax = 1.0e-9;  // avoid division by zero
  boost::lock_guard<boost::mutex> l(sensMutex_);
  for (size_t i = 0; i < motorPorts.size(); ++i) {
    const unsigned char motorPort = motorPorts[i];
    if (motorPort > 3) {
      return false;
    }

    // get the current angles as a start
    if (controlModes_[motorPort] != Velocity
        && controlModes_[motorPort] != Position) {
      filteredAngleReferences_[motorPort].store(motorAngles_[motorPort]);
    }

    // compute distance and remember maximum
    const double currentPosition = filteredAngleReferences_[motorPort];
    deltaPositions[i] = angleReferences[i] - currentPosition;
    if (fabs(deltaPositions[i]) > deltaPositionsMax) {
      deltaPositionsMax = fabs(deltaPositions[i]);
    }
  }

  for (size_t i = 0; i < motorPorts.size(); ++i) {
    const unsigned char motorPort = motorPorts[i];
    double angleReference = angleReferences[i];
    double rotationSpeedReference = rotationSpeedReferences[i];

    if (rotationSpeedReference < 0.0) {
      rotationSpeedReference = -rotationSpeedReference;  // ensure positive
    }

    // compute scaling so the motions become synchronised:
    const double scaling = fabs(deltaPositions[i]) / deltaPositionsMax;

    // store angle reference
    inDeceleration_[motorPort] = false;
    referenceReached_[motorPort] = false;
    angleReferences_[motorPort] = angleReference;

    // get the needed direction...
    // first compute the time and distance needed to stop
    const double currentPosition = filteredAngleReferences_[motorPort];
    const double deltaPosition = angleReference - currentPosition;
    const double currentSpeed = filteredRotationSpeedReferences_[motorPort];
    mutex_[motorPort].lock();
    double acc = motorAngleControllerParameters_[motorPort]
        .maxRotationAcceleration * scaling;
    mutex_[motorPort].unlock();
    double dec = -acc;
    if (currentSpeed < 0.0) {
      dec = -dec;
    }
    const double dt_stop = -currentSpeed / dec;
    const double ds_stop = -0.5 * dec * dt_stop * dt_stop;

    // we use a negative slope, if
    double sign = 1.0;
    if ((ds_stop + currentPosition) > angleReference) {
      sign = -1.0;
    }
    rotationSpeedReferences_[motorPort] = sign * rotationSpeedReference
        * scaling;  // set speed reference

    //std::cout << "rotationSpeedReferences_[" << int(motorPort) << "]"
    // << rotationSpeedReferences_[motorPort] << std::endl;

    // set the acceleration and deceleration
    if (currentSpeed < rotationSpeedReferences_[motorPort]) {
      filteredMaxRotationAccelerations_[motorPort] = acc;
    } else {
      filteredMaxRotationAccelerations_[motorPort] = -acc;
    }
    filteredMaxRotationDecelerations_[motorPort] = -sign * acc;

    //std::cout << "filteredMaxRotationDecelerations " << filteredMaxRotationDecelerations_[motorPort] << std::endl;
    controlModes_[motorPort] = Position;  // reset mode
  }
  return true;
}

/// increase a controller angle reference -- overrides low-level setMotorPwm:
bool Interface::increaseMotorAngleReference(unsigned char motorPort,
                                            double angleReferenceIncrease)
{
  if (motorPort > 3) {
    return false;
  }
  // get the current angles as a start
  if (controlModes_[motorPort] != Velocity
      && controlModes_[motorPort] != Position) {
    boost::lock_guard<boost::mutex> l(sensMutex_);
    filteredAngleReferences_[motorPort].store(motorAngles_[motorPort]);
  }
  // increase
  const double angleRef = filteredAngleReferences_[motorPort];
  return setMotorAngleReference(motorPort, angleRef + angleReferenceIncrease);
}
bool Interface::increaseMotorAngleReference(unsigned char motorPort,
                                            double angleReferenceIncrease,
                                            double rotationSpeedReference)
{
  if (motorPort > 3) {
    return false;
  }
  // get the current angles as a start
  if (controlModes_[motorPort] != Velocity
      && controlModes_[motorPort] != Position) {
    boost::lock_guard<boost::mutex> l(sensMutex_);
    filteredAngleReferences_[motorPort].store(motorAngles_[motorPort]);
  }
  // increase
  const double angleRef = filteredAngleReferences_[motorPort];
  return setMotorAngleReference(motorPort, angleRef + angleReferenceIncrease,
                                rotationSpeedReference);
}
/// increase controller speed references -- overrides low-level setMotorPwm.
/// this version guarantees synchronous
bool Interface::increaseMotorAngleReferences(
    const std::vector<unsigned char> & motorPorts,
    const std::vector<double> & angleReferenceIncreases)
{
  std::vector<double> angleReferences;
  sensMutex_.lock();
  for (size_t i = 0; i < motorPorts.size(); ++i) {
    const unsigned char motorPort = motorPorts[i];
    // get the current angles as a start
    if (controlModes_[motorPort] != Velocity
        && controlModes_[motorPort] != Position) {
      filteredAngleReferences_[motorPort].store(motorAngles_[motorPort]);
    }
    // increase
    const double angleRef = filteredAngleReferences_[motorPort];
    angleReferences.push_back(angleRef + angleReferenceIncreases[motorPort]);
  }
  sensMutex_.unlock();
  return setMotorAngleReferences(motorPorts, angleReferences);
}
bool Interface::increaseMotorAngleReferencesPython(
  const boost::python::list & motorPortsList,
  const boost::python::list & angleReferenceIncreasesList)
{
  std::vector<unsigned char> motorPorts;
  python_list_to_std_vector(motorPortsList, motorPorts);
  std::vector<double> angleReferenceIncreases;
  python_list_to_std_vector(angleReferenceIncreasesList, angleReferenceIncreases);
  return increaseMotorAngleReferences(motorPorts, angleReferenceIncreases);
}
bool Interface::increaseMotorAngleReferencesPython(
  const boost::python::list & motorPortsList,
  const boost::python::list & angleReferenceIncreasesList,
  const boost::python::list & rotationSpeedReferencesList)
{
  std::vector<unsigned char> motorPorts;
  python_list_to_std_vector(motorPortsList, motorPorts);
  std::vector<double> angleReferenceIncreases;
  python_list_to_std_vector(angleReferenceIncreasesList, angleReferenceIncreases);
  std::vector<double> rotationSpeedReferences;
  python_list_to_std_vector(rotationSpeedReferencesList, rotationSpeedReferences);
  return increaseMotorAngleReferences(motorPorts, angleReferenceIncreases, rotationSpeedReferences);
}
bool Interface::increaseMotorAngleReferences(
    const std::vector<unsigned char> & motorPorts,
    const std::vector<double> & angleReferenceIncreases,
    const std::vector<double> & rotationSpeedReferences)
{
  std::vector<double> angleReferences;
  sensMutex_.lock();
  for (size_t i = 0; i < motorPorts.size(); ++i) {
    const unsigned char motorPort = motorPorts[i];
    // get the current angles as a start
    if (controlModes_[motorPort] != Velocity
        && controlModes_[motorPort] != Position) {
      filteredAngleReferences_[motorPort].store(motorAngles_[motorPort]);
    }
    // increase
    const double angleRef = filteredAngleReferences_[motorPort];
    angleReferences.push_back(angleRef + angleReferenceIncreases[motorPort]);
  }
  sensMutex_.unlock();
  return setMotorAngleReferences(motorPorts, angleReferences,
                                 rotationSpeedReferences);
}

/// query the current setpoint:
bool Interface::getMotorAngleReference(unsigned char motorPort,
                                       double & angleReference)
{
  if (motorPort > 3) {
    return false;
  }
  if (controlModes_[motorPort] == Position) {
    angleReference = angleReferences_[motorPort];
    return true;
  }
  return false;
}
boost::python::list Interface::getMotorAngleReferencesPython(
  const boost::python::list & motorPorts)
{
  boost::python::list l;
  for(int i=0; i<boost::python::len(motorPorts); i++)
  {
    unsigned char motorPort = boost::python::extract<unsigned char>(motorPorts[i]);
    if(motorPort > 3)
    {
      return boost::python::list();
    }
    if (controlModes_[motorPort] == Position) {
      l.append(angleReferences_[motorPort].load() );
    }
    else
    {
      return boost::python::list();
    }
  }
  return l;
}
/// query the current (actual) motor speed
bool Interface::getMotorAngle(unsigned char motorPort, double & angle,
                              timestamp_t &timestamp)
{
  if (motorPort > 3) {
    return false;
  }
  if (controlModes_[motorPort] == Disabled) {
    return false;
  }
  sensMutex_.lock();
  angle = motorAngles_[motorPort];
  timestamp = motorTimestamp;
  sensMutex_.unlock();
  return true;
}
boost::python::tuple Interface::getMotorAnglePython(unsigned char motorPort)
{
  double angle;
  timestamp_t timestamp;
  bool success = getMotorAngle(motorPort, angle, timestamp);
  if (success)
    return boost::python::make_tuple(angle, timestamp);
  else
    return boost::python::tuple();
}
bool Interface::getMotorAngle(unsigned char motorPort, double & angle)
{
  timestamp_t timestamp;
  return getMotorAngle(motorPort, angle, timestamp);
}

bool Interface::getMotorAngles(const std::vector<unsigned char>& motorPorts,
                               std::vector<double> & angles,
                               std::vector<timestamp_t> & timestamps)
{
  angles.resize(motorPorts.size());
  timestamps.resize(motorPorts.size());
  boost::lock_guard<boost::mutex> l(sensMutex_);
  for (size_t i = 0; i < motorPorts.size(); ++i) {
    const unsigned char motorPort = motorPorts.at(i);
    if (motorPort > 3) {
      return false;
    }
    if (controlModes_[motorPort] == Disabled) {
      return false;
    }
    angles[i] = motorAngles_[motorPort];
    timestamps[i] = motorTimestamp;
  }
  return true;
}

boost::python::list Interface::getMotorAnglesPython(
  const boost::python::list &motorPorts)
{
  boost::python::list values;
  boost::lock_guard<boost::mutex> l(sensMutex_);
  for (size_t i = 0; i < boost::python::len(motorPorts); ++i) {
    const unsigned char motorPort = boost::python::extract<unsigned char>(motorPorts[i]);
    if (motorPort > 3) {
      return boost::python::list();
    }
    if (controlModes_[motorPort] == Disabled) {
      return boost::python::list();
    }
    values.append(boost::python::make_tuple(
        motorAngles_[motorPort],
        motorTimestamp.load()
      ));
  }
  return values;
}

bool Interface::getMotorAngles(const std::vector<unsigned char>& motorPorts,
                               std::vector<double> & angles)
{
  std::vector<timestamp_t> unused;
  return getMotorAngles(motorPorts, angles, unused);
}

bool Interface::motorAngleReferenceReached(unsigned char motorPort)
{
  if (motorPort > 3) {
    return false;
  }
  if (controlModes_[motorPort] != Position) {
    std::cout << "motorAngleReferenceReached will only return true if in Position control mode." << std::endl;
    return false;
  }
  return referenceReached_[motorPort];
}

bool Interface::motorAngleReferencesReachedPython(
  const boost::python::list & motorPorts)
{
  bool destinationReached = true;
  for(int i=0; i<boost::python::len(motorPorts); i++)
  {
    unsigned char motorPort = boost::python::extract<unsigned char>(motorPorts[i]);
    if(motorPort > 3)
    {
      std::cout << "motorAngleReferencesReached: ERROR: motorPort out of range." << std::endl;
      return false;
    }
    if (controlModes_[motorPort] != Position) {
      std::cout << "motorAngleReferencesReached: ERROR: not in Position control mode." << std::endl;
      return false;
    }
    destinationReached = destinationReached && referenceReached_[motorPort];
  }
  return destinationReached;
}

bool Interface::filterVelocityMode(double dt, unsigned char motorPort)
{
  boost::lock_guard<boost::mutex> l(mutex_[motorPort]);
  // filter with max. acceleration
  const double speedDiff = rotationSpeedReferences_[motorPort]
      - filteredRotationSpeedReferences_[motorPort];
  const double acc = filteredMaxRotationAccelerations_[motorPort].load();
  if (fabs(speedDiff) / dt
      > motorAngleControllerParameters_[motorPort].maxRotationAcceleration) {
    filteredRotationSpeedReferences_[motorPort].store(
        filteredRotationSpeedReferences_[motorPort].load() + dt * acc);
    // integrate
    filteredAngleReferences_[motorPort].store(
        filteredAngleReferences_[motorPort].load()
            + dt * filteredRotationSpeedReferences_[motorPort]
            + 0.5 * dt * dt * acc);
  } else {
    filteredRotationSpeedReferences_[motorPort].store(
        rotationSpeedReferences_[motorPort]);
    // integrate
    filteredAngleReferences_[motorPort].store(
        filteredAngleReferences_[motorPort].load()
            + dt * filteredRotationSpeedReferences_[motorPort]);
  }

  /*std::cout << "dt: " << dt << "speed: "
   << filteredRotationSpeedReferences_[motorPort] << ", pos: "
   << filteredAngleReferences_[motorPort] << std::endl;*/
  return true;
}

bool Interface::filterPositionMode(double dt, unsigned char motorPort)
{

  boost::lock_guard<boost::mutex> l(mutex_[motorPort]);
  const double currentSpeed = filteredRotationSpeedReferences_[motorPort];
  const double speedDiff = currentSpeed - rotationSpeedReferences_[motorPort];
  const double acc = filteredMaxRotationAccelerations_[motorPort].load();
  double filteredRotationSpeedReference;
  double filteredAngleReference;
  if (fabs(speedDiff) / dt
      > fabs(filteredMaxRotationAccelerations_[motorPort])) {
    // full speed acceleration
    filteredRotationSpeedReference = filteredRotationSpeedReferences_[motorPort]
        .load() + dt * acc;
    // integrate
    filteredAngleReference = filteredAngleReferences_[motorPort].load()
        + dt * filteredRotationSpeedReference + 0.5 * dt * dt * acc;
  } else {
    // reference speed
    filteredRotationSpeedReference = rotationSpeedReferences_[motorPort];
    // integrate
    filteredAngleReference = filteredAngleReferences_[motorPort].load()
        + dt * filteredRotationSpeedReference;
  }

  // check, if deceleration is needed - make it stop in any direction to be on the safe side
  const double angle_ref = angleReferences_[motorPort];
  double dec = filteredMaxRotationDecelerations_[motorPort].load();

  const double dt_stop1 = -filteredRotationSpeedReference / dec;
  if(dt_stop1 < 0.0) {
    dec = -dec;
  }
  const double ds_stop1 = -0.5 * dec * dt_stop1 * dt_stop1;
  const double currentPosition1 = filteredAngleReference;
  const double deltaPosition1 = angle_ref - currentPosition1;
  const double stopAngle1 = currentPosition1 + ds_stop1;

  const double dt_stop0 = -filteredRotationSpeedReferences_[motorPort] / dec;
  if(dt_stop0 < 0.0) {
    dec = -dec;
  }
  const double ds_stop0 = -0.5 * dec * dt_stop0 * dt_stop0;
  const double currentPosition0 = filteredAngleReferences_[motorPort];
  const double deltaPosition0 = angle_ref - currentPosition0;
  const double stopAngle0 = currentPosition0 + ds_stop0;

  //std::cout << stopAngle0 << " " << stopAngle1 << " " << angle_ref << std::endl;

  if ((angle_ref - stopAngle1) * (angle_ref - stopAngle0) <= 0.0) {
    inDeceleration_[motorPort] = true;
  }
  if (inDeceleration_[motorPort]) {
    // make sure no more acceleration
    filteredMaxRotationAccelerations_[motorPort] = 0.0;
    // recompute deceleration
    const double speed = filteredRotationSpeedReferences_[motorPort].load();
    const double diff = (angleReferences_[motorPort]
        - filteredAngleReferences_[motorPort].load());
    if (fabs(diff) < 1.0e-12) {
      // stop
      filteredRotationSpeedReference = 0.0;
      filteredAngleReference = angle_ref;
    } else {
      dec = -speed * speed * 0.5 / diff;
      // recompute speed
      filteredRotationSpeedReference = speed + dt * dec;
      // recompute angle reference
      filteredAngleReference = filteredAngleReferences_[motorPort].load()
          + dt * filteredRotationSpeedReference + 0.5 * dt * dt * dec;
    }
  }
  // check stopping
  const double last_error = angle_ref
      - filteredAngleReferences_[motorPort].load();
  const double this_error = angle_ref - filteredAngleReference;
  if (inDeceleration_[motorPort]
      && filteredRotationSpeedReference
          * filteredRotationSpeedReferences_[motorPort] <= 0.0) {
    filteredRotationSpeedReference = 0.0;
    filteredAngleReference = angle_ref;
  }
  if (inDeceleration_[motorPort]
      && fabs(angle_ref - filteredAngleReference) < 0.5 / TICKS_PER_RAD) {
    filteredRotationSpeedReference = 0.0;
    filteredAngleReference = angle_ref;
  }

  // update
  filteredRotationSpeedReferences_[motorPort].store(
      filteredRotationSpeedReference);
  filteredAngleReferences_[motorPort].store(filteredAngleReference);

  return false;
}

double Interface::deadbandCompensatedPwm(unsigned char motorPort, double pwm)
{
  const double minPwm = motorAngleControllerParameters_[motorPort].minPWM;
  if(fabs(pwm)>=2.0*minPwm)
  {
    return pwm;
  }
  else
  {
    if (pwm>1.0e-12) return 1.0/(4.0*minPwm)*pwm*pwm+minPwm;
    if (pwm<-1.0e-12) return -1.0/(4.0*minPwm)*pwm*pwm-minPwm;
  }
  return 0.0;
  //return ((speedReference>0) - (speedReference<0)) * motorAngleControllerParameters_[motorPort].minPWM 
  //    + motorAngleControllerParameters_[motorPort].feedForwardGain * speedReference;
}

bool Interface::controlMotors(timestamp_t timestamp)
{
  bool success = true;

  // compute time situation
  uint64_t now = CurrentTickUs();
  double dt = 0.0;
  if (lastTime_ != 0) {
    dt = double(now - lastTime_) / 1.0e6;
  }
  lastTime_ = now;

  if(loggingEnabled)
  {
    logFile << std::setprecision(7) << (double)timestamp*1.0e-9 << "\t";
  }

  for (size_t i = 0; i < 4; ++i) {
    ControlMode controlMode = controlModes_[i];
    // update angle NO LONGER DONE HERE
    //motorAngles_[i] = double(BrickPi.Encoder[i]) / TICKS_PER_RAD;
    //motorTimestamp = timestamp;
    switch (controlMode) {
      case Disabled:
        // don't do anything
        break;
      case Manual:
        // simply send pwm:
        BrickPi.MotorSpeed[i] = pwmReferences_[i].load();
        break;
      case Velocity: {
        // apply reference filtering
        filterVelocityMode(dt, i);

        // compute error
        double error = filteredAngleReferences_[i].load() - motorAngles_[i];

        if(loggingEnabled)
        {
          logFile << std::setprecision(9)
                  << filteredAngleReferences_[i].load() 
                  << "\t" << motorAngles_[i] 
                  << "\t";
        }

        // check if reference velocity was reached
        double output = 0.0;
        if ((fabs(filteredRotationSpeedReferences_[i] - rotationSpeedReferences_[i])<1.0e-12) &&
            (fabs(error) < 2.5/TICKS_PER_RAD)) {
          referenceReached_[i] = true;
          pidControllers_[i].reset(); // for continuity
        } else {
          // compute PID output
          output = pidControllers_[i].computeOutput(error, now * 1e3);

          // add feed-forward
          output += filteredRotationSpeedReferences_[i].load()*
              motorAngleControllerParameters_[i].feedForwardGain;

          // dead-band compensation
          output = deadbandCompensatedPwm(i,output);
        }

        // set output -- saturate again for safety
        if (output > 255) {
          output = 255;
        }
        if (output < -255) {
          output = -255;
        }

        //std::cout << "i="<< i
        //    << ", error : " << error
        //    << ", actual angle : " << motorAngles_[i]
        //    << ", angle reference : " << filteredAngleReferences_[i].load()
        //    << ", speed reference : " << this->filteredRotationSpeedReferences_[i].load() << std::endl;

        BrickPi.MotorSpeed[i] = int(output);

        break;
      }
      case Position: {
        // apply reference filtering
        filterPositionMode(dt, i);

        // compute error output
        double error = filteredAngleReferences_[i].load() - motorAngles_[i];

        if(loggingEnabled)
        {
          logFile << std::setprecision(9)
                  << filteredAngleReferences_[i].load() 
                  << "\t" << motorAngles_[i] 
                  << "\t";
        }

        // check if reference velocity was reached
        double output = 0.0;
        if ((fabs(filteredRotationSpeedReferences_[i]) < 1.0e-12) &&
            (fabs(filteredAngleReferences_[i] - angleReferences_[i]) < 1.0e-5) &&
            (fabs(error) < 2.5/TICKS_PER_RAD)) {
          referenceReached_[i] = true;
          pidControllers_[i].reset(); // for continuity
        } else {
          // compute PID output
          output = pidControllers_[i].computeOutput(error, now * 1e3);

          // add feed-forward
          output += filteredRotationSpeedReferences_[i].load()*
                      motorAngleControllerParameters_[i].feedForwardGain;

          // dead-band compensation
          //std::cout<<output<<"->";
          output = deadbandCompensatedPwm(i,output);
          //std::cout<<output<<std::endl;
        }

        // set output -- saturate again for safety
        if (output > 255) {
          output = 255;
        }
        if (output < -255) {
          output = -255;
        }

        BrickPi.MotorSpeed[i] = int(output);
        //std::cout << "i=" << i << ", error : " << error << ", actual angle : "
        //          << motorAngles_[i] << ", angle reference : "
        //          << filteredAngleReferences_[i].load()
        //          << ", speed reference : "
        //          << filteredRotationSpeedReferences_[i].load()
        //          << std::endl;

        break;
      }
      default:
        break;
    }    
  }
  if(loggingEnabled)
  {
    logFile << "\n";
  }
  return success;
}

int Interface::controlAndSensingLoop()
{

  std::cout << "controlAndSensingLoop: running" << std::endl;

  boost::timer::cpu_timer intervalTimer;
  boost::timer::nanosecond_type startTime, lastUltrasonicUpdate = 0;
  intervalTimer.start();
  while (!shutdown_.load()) {
    startTime = intervalTimer.elapsed().wall;

    const bool getUltrasonic = ultrasonicEnabled.load()
        && ((startTime / 1000 - lastUltrasonicUpdate / 1000)
            > ULTRASONIC_INTERVAL_MILLISECONDS * 1000);

    timestamp_t timestamp;
    updateValues(getUltrasonic, timestamp);

    // handle the control of all motors
    controlMotors(timestamp);

    if (getUltrasonic)
      lastUltrasonicUpdate = timestamp;

    // sleep for the rest of the interval
    boost::this_thread::sleep(
        boost::posix_time::microseconds(
            (CONTROL_INTERVAL_SECONDS * 1.0e6
                - (intervalTimer.elapsed().wall / 1000 - startTime / 1000))));
  }

  std::cout << "controlAndSensingLoop: finished" << std::endl;
  return 0;
}

bool Interface::updateSensorValues(bool getUltrasonic, timestamp_t timestamp)
{
  boost::lock_guard<boost::mutex> l(sensMutex_);
  for (int i = 0; i < 4; i++) {
    SensorType type = sensorTypes_[i];
    if(type!=SENSOR_ULTRASONIC || (type==SENSOR_ULTRASONIC && getUltrasonic)) {      
    //if (BrickPi.Sensor[i] >= 0) {
      sensorValues_[i] = sensorValue_t(BrickPi.Sensor[i], timestamp);
    }
    motorAngles_[i] = double(BrickPi.Encoder[i]) / TICKS_PER_RAD;
    motorTimestamp = timestamp;
  }
  return true;
}

bool Interface::getSensorValue(unsigned char sensorPort, long &value,
                               timestamp_t &timestamp)
{
  if (sensorPort > 3) {
    return false;
  }
  if (sensorTypes_[sensorPort] == SENSOR_NONE) {
    return false;
  }
  sensMutex_.lock();
  sensorValue_t sense = sensorValues_[sensorPort];
  sensMutex_.unlock();
  if (sense.value < 0) {
    return false;
  } else {
    value = sense.value;
    timestamp = sense.timestamp;
    return true;
  }
}

bool Interface::getSensorValue(unsigned char sensorPort, long &value)
{
  timestamp_t timestamp;
  return getSensorValue(sensorPort, value, timestamp);
}

boost::python::tuple Interface::getSensorValuePython(unsigned char sensorPort)
{
  long value;
  timestamp_t timestamp;
  bool success = getSensorValue(sensorPort, value, timestamp);
  if (success)
    return boost::python::make_tuple(value, timestamp);
  else
    return boost::python::tuple();
}

bool Interface::setupSensors()
{
  int result = 1;
  boost::lock_guard<boost::mutex> l(commMutex_);
  while (result) {
    result = BrickPiSetupSensors();  //Set up the properties of sensors for the BrickPi
  }
  return true;
}

bool Interface::updateValues(bool getUltrasonic, timestamp_t & timestamp)
{
  int result = 1;
  boost::lock_guard<boost::mutex> l(commMutex_);
  while (result) {
    result = BrickPiUpdateValues(getUltrasonic);
    boost::this_thread::sleep(boost::posix_time::milliseconds(1));
  }
  timestamp = timer.elapsed().wall;

  // handle the sensors and encoders
  updateSensorValues(getUltrasonic, timestamp);

  return true;
}

bool Interface::startLogging(std::string fileName)
{
  if(loggingEnabled)
  {
    std::cout << "Logging already enabled!" << std::endl;
    return false;
  }
  logFile.open(fileName.c_str());
  if(logFile.is_open())
  {
    loggingEnabled = true;
    return true;
  }
  else
  {
    std::cout << "Error opening logfile: " << fileName << std::endl;
    return false;
  }
}

bool Interface::stopLogging()
{
  if(loggingEnabled)
  {
    loggingEnabled = false;
    if(logFile.is_open())
    {
      logFile.close();
      return true;
    }
    else
    {
      std::cout << "Logging enabled but logfile not open :S" << std::endl;
      return false;
    }
  }
  else
  {
    std::cout << "Logging already disabled" << std::endl;
    return false;
  }
}

}  // namespace brickpi
