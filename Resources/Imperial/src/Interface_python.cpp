#include <boost/python.hpp>
#include "Interface.hpp"
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>

using namespace brickpi;
using namespace boost::python;

//bool (Interface::*getSensorValue1)(unsigned char, long&) = &Interface::getSensorValue;
//bool (Interface::*getSensorValue2)(unsigned char, long&, Interface::timestamp_t&) = &Interface::getSensorValue;


bool (Interface::*setMotorAngleReference1)(unsigned char, double) = 
	&Interface::setMotorAngleReference;
bool (Interface::*setMotorAngleReference2)(unsigned char, double, double) = 
	&Interface::setMotorAngleReference;

bool (Interface::*setMotorAngleReferences1)(
	const boost::python::list&, 
	const boost::python::list&) = 
		&Interface::setMotorAngleReferencesPython;
bool (Interface::*setMotorAngleReferences2)(
	const boost::python::list&, 
	const boost::python::list&,
	const boost::python::list&) = 
		&Interface::setMotorAngleReferencesPython;

bool (Interface::*increaseMotorAngleReference1)(unsigned char, double) = 
	&Interface::increaseMotorAngleReference;
bool (Interface::*increaseMotorAngleReference2)(unsigned char, double, double) = 
	&Interface::increaseMotorAngleReference;

bool (Interface::*increaseMotorAngleReferences1)(
	const boost::python::list&, 
	const boost::python::list&) = 
		&Interface::increaseMotorAngleReferencesPython;
bool (Interface::*increaseMotorAngleReferences2)(
	const boost::python::list&, 
	const boost::python::list&,
	const boost::python::list&) = 
		&Interface::increaseMotorAngleReferencesPython;


BOOST_PYTHON_MODULE(brickpi)
{
  // This will enable user-defined docstrings and python signatures,
  // while disabling the C++ signatures
  docstring_options local_docstring_options(true, false, false);

	class_<std::vector<unsigned char> >("vector_uchar")
        .def(vector_indexing_suite<std::vector<unsigned char> >() );
    class_<std::vector<double> >("vector_double")
        .def(vector_indexing_suite<std::vector<double> >() );
    class_<std::vector<Interface::timestamp_t> >("vector_timestamp")
        .def(vector_indexing_suite<std::vector<Interface::timestamp_t> >() );


	enum_<Interface::SensorType>("SensorType")
		.value("SENSOR_NONE", Interface::SENSOR_NONE)
		.value("SENSOR_TOUCH", Interface::SENSOR_TOUCH)
		.value("SENSOR_ULTRASONIC", Interface::SENSOR_ULTRASONIC)
		;
		

	scope main_scope = class_<Interface, boost::noncopyable>(
	    "Interface",
	    "BrickPi Interface class.\n"
	    "We mirror the functionalities provided by the manufacturer in C++ style \n"
	    "and additionally provide (somewhat) low-level speed and position control\n"
	    "for the motors." )
		.def("initialize", &Interface::initialize,
		     "This starts a separate thread that continuously polls the activated \n"
		     "sensors as well as controls the motors that were started.\n"
		     "\n"
		     ":returns: true on success")
		.def("terminate", &Interface::terminate,
		     "Softly stop all motors and sensors, stop the polling and control thread.\n"
		     "\n"
		     ":returns: true on success")
		.def("emergencyStop", &Interface::emergencyStop,
		     "Immediately stop all motors, stop the polling and control thread (bad\n"
		     "for the hardware).\n"
		     "\n"
		     ":returns: true on success")
		.def("motorEnable", &Interface::motorEnable,
		     "Enable individual motor.\n"
		     "\n"
		     ":param motorPort: motor port [0..3]\n"
         ":returns: true on success")
		.def("motorDisable", &Interface::motorDisable,
		     "Disable individual motor.\n"
		     "\n"
		     ":param motorPort: motor port [0..3]\n"
		     ":returns: true on success")
		.def("sensorEnable", &Interface::sensorEnable,
		     "Enable individual sensor.\n"
		     "\n"
         ":param sensorPort: sensor port [0..4]\n"
         ":returns: true on success")
		.def("sensorDisable", &Interface::sensorDisable,
		     "Disable individual sensor with.\n"
		     "\n"
         ":param sensorPort: sensor port [0..4]\n"
         ":returns: true on success")
		.def("getSensorValue", &Interface::getSensorValuePython,
		     "Read a sensor value (thread-safe). \n"
		     "\n"
		     ":param sensorPort: sensor port [0..4]\n"
		     ":returns: tuple of sensor value and timstamp [nsec] or empty on failure")
		.def("setMotorPwm", &Interface::setMotorPwm,
		     "Puts the motor motorPort into manual control mode and directly applies \b"
		     "the PWM value.\n"
		     "\n"
		     ":param pwm: PWM value [-255,255]\n"
		     ":returns true, if successful.")
		.def("setMotorAngleControllerParameters", &Interface::setMotorAngleControllerParameters,
		     "Set the angle controller parameters for a motor.\n"
		     "\n"
		     ":returns: true on success")
		.def("setMotorRotationSpeedReference", &Interface::setMotorRotationSpeedReference,
		     "Put a motor to speed control mode and set its rotation speed\n"
		     "reference. You may want to check with \n"
		     "bool motorRotationSpeedReferenceReached(motorPort), when the reference is \n"
		     "reached.\n"
		     "\n"
		     ":param motorPort: motor port [0..3]\n"
		     ":param rotationSpeedReference: rotation speed references [rad/s]\n"
		     ":returns: true on success")
		.def("setMotorRotationSpeedReferences", &Interface::setMotorRotationSpeedReferencesPython,
		     "Put several motors to speed control mode and set their rotation speed\n"
		     "references. You may want to check with \n"
		     "bool motorRotationSpeedReferenceReached(motorPort), when the reference is \n"
		     "reached.\n"
		     "\n"
		     ":param motorPorts: list of motor ports [0..3]\n"
		     ":param rotationSpeedReferences: list of rotation speed references [rad/s]\n"
		     ":returns: true on success")
		//.def("getMotorRotationSpeedReference", &Interface::getMotorRotationSpeedReference)
		.def("motorRotationSpeedReferenceReached", &Interface::motorRotationSpeedReferenceReached,
		     "Checks, if the previously set rotation speed reference was reached.\n"
		     "Note: you must set reference rotation speed(s) before.\n"
		     "\n"
		     ":returns: true on success")
		.def("setMotorAngleReference", setMotorAngleReference1,
		     "Put a motor to position control mode and set a reference angle that\n"
		     "should be reached.\n"
		     "\n"
		     ":param motorPort: motor port [0..3]\n"
		     ":param angleReference: angle reference [rad]\n"
		     ":returns: true on success")
		.def("setMotorAngleReference", setMotorAngleReference2,
         "Put a motor to position control mode and set a reference angle that\n"
         "should be reached, while not exceeding a maximum rotation speed.\n"
         "\n"
         ":param motorPort: motorPort [0..3]\n"
         ":param angleReference: angle reference [rad]\n"
         ":param rotationSpeedReference: (maximum) rotation speed [rad/s]\n"
         ":returns: true on success")
		.def("setMotorAngleReferences", setMotorAngleReferences1,
         "Put motors to position control mode and set a reference angle that\n"
         "should be reached.\n"
         "\n"
         ":param motorPorts: list of motor ports [0..3]\n"
         ":param angleReference: corresp. list of angle references [rad]\n"
         ":returns: true on success")
		.def("setMotorAngleReferences", setMotorAngleReferences2,
		     "Put motors to position control mode and set a reference angle that\n"
		     "should be reached.\n"
		     "\n"
		     ":param motorPorts: list of motor ports [0..3]\n"
		     ":param angleReference: corresp. list of angle references [rad]\n"
		     ":param rotationSpeedReferences: list of (maximum) rotation speeds [rad/s]\n"
		     ":returns: true on success")
		.def("increaseMotorAngleReference", increaseMotorAngleReference1,
		     "Put motors to position control mode and increase the reference angle\n"
		     "that should be reached.\n"
		     "\n"
		     ":param motorPort: motor port [0..3]\n"
		     ":param angleReferenceIncrease: angle reference increase [rad]\n"
		     ":returns: true on success")
		.def("increaseMotorAngleReference", increaseMotorAngleReference2,
         "Put motors to position control mode and increase the reference angle\n"
         "that should be reached, while not exceeding a maximum rotation speed.\n"
         "\n"
         ":param motorPort: motor port [0..3]\n"
         ":param angleReferenceIncrease: angle reference increase [rad]\n"
         ":param rotationSpeedReference: (maximum) rotation speed [rad/s]\n"
         ":returns: true on success")
		.def("increaseMotorAngleReferences", increaseMotorAngleReferences1,
		     "Put motors to position control mode and increase the reference angle\n"
		     "that should be reached.\n"
		     "\n"
		     ":param motorPorts: list of motor ports [0..3]\n"
		     ":param angleReferenceIncreases: list of angle reference increases [rad]\n"
		     ":returns: true on success")
		.def("increaseMotorAngleReferences", increaseMotorAngleReferences2,
		     "Put motors to position control mode and increase the reference angle\n"
		     "that should be reached, while not exceeding a maximum rotation speed.\n"
		     "\n"
		     ":param motorPorts: list of motor ports [0..3]\n"
		     ":param angleReferenceIncreases: list of angle reference increases [rad]\n"
         ":param rotationSpeedReference: list of (maximum) rotation speed [rad/s]\n"
		     ":returns: true on success")
		//.def("getMotorAngleReference", &Interface::getMotorAngleReference)
		.def("getMotorAngleReferences", &Interface::getMotorAngleReferencesPython,
		     "Get the currently set motor angle references. Note: you must set \n"
		     "reference rotation speed(s) before.\n"
		     "\n"
		     ":param motorPorts: list of motor ports [0..3]\n"
		     ":returns: list of reference angles [rad], empty on failure")
		.def("getMotorAngle", &Interface::getMotorAnglePython,
		     "Get the current motor angle.\n"
		     "\n"
		     ":param motorPort: list of motor ports [0..3]\n"
		     ":returns: tuple of angle [rad] and timestamp [nsec], empty on failure")
		.def("getMotorAngles", &Interface::getMotorAnglesPython,
		     "Get the current motor angles.\n"
		     "\n"
		     ":param motorPorts: list of motor ports [0..3]\n"
		     ":returns: list of tuples of angle [rad] and timestamp [nsec], empty on failure")
		.def("motorAngleReferenceReached", &Interface::motorAngleReferenceReached,
		     "Determine, if a motor has reached its reference angle. Note: \n"
		     "you must set a reference angle before.\n"
		     "\n"
		     ":param motorPort: motor port [0..3]\n"
		     ":returns: true, if reached")
		.def("motorAngleReferencesReached", &Interface::motorAngleReferencesReachedPython,
		     "Determine, if motors have reached their reference angles. Note: \n"
		     "you must set reference angles before.\n"
		     "\n"
		     ":param motorPorts: list of motor ports [0..3]\n"
		     ":returns: true, if all angles have reached their references")
		.def("startLogging", &Interface::startLogging,
		     "Start logging of timestamp, filtered angle references and actual angles.\n"
		     "\n"
		     ":param filename: full path to your logfile\n"
		     ":returns true on succes")
		.def("stopLogging", &Interface::stopLogging,
		     "Stop logging of timestamp, filtered angle references and actual angles.\n"
		     "\n"
		     ":returns true on succes")
		;

	//These classes will be defined within the scope of Interface
	class_<Interface::MotorAngleControllerParameters>("MotorAngleControllerParameters",
	    "This is a simple struct to store all motor parameters relevant\n"
	    "for controlling it.")
		.def_readwrite("maxRotationAcceleration", &Interface::MotorAngleControllerParameters::maxRotationAcceleration,
		               "Maximum admissible rotational acceleration [rad/s^2]")
		.def_readwrite("maxRotationSpeed", &Interface::MotorAngleControllerParameters::maxRotationSpeed,
		               "Maximum admissible rotational speed [rad/s]")
		.def_readwrite("feedForwardGain", &Interface::MotorAngleControllerParameters::feedForwardGain,
		               "Fead-forward component added to as \n"
		               "(filtered desired speed)*feedForwardGain")
		.def_readwrite("minPWM", &Interface::MotorAngleControllerParameters::minPWM,
		               "Minimum PWM that will always be set \n"
		               "(overcoming friction dead-band)")
		.def_readwrite("pidParameters", &Interface::MotorAngleControllerParameters::pidParameters,
		               "This contains all PID parameters")
		;

	class_<PidController::Parameters>("PidParameters",
	    "This is a simple struct to store all PID parameters")
		.def_readwrite("minOutput", &PidController::Parameters::minOutput,
		               "Minimum output to be generated (typically -255)")
		.def_readwrite("maxOutput", &PidController::Parameters::maxOutput,
                   "Maximum output to be generated (typically 255)")
		.def_readwrite("k_p", &PidController::Parameters::k_p,
		               "Proportional gain (P)")
		.def_readwrite("k_i", &PidController::Parameters::k_i,
		               "Integral gain (I)")
		.def_readwrite("K_d", &PidController::Parameters::k_d,
		               "Differential gain (D)")
		;


}
