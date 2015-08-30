//==========================================================================================================================================
//Copyright (C) 2015 The Botsmiths: FRC Team 4309
//Author(s): Tanner Mickelson (supertanner@gmail.com)
//License: GNU GPLv2 see LICENSE file for full text.
//
//Botsmiths Single Header Utility Library (tentative title while I think of a cool acronym)
//The Botsmiths Single Header Utility Library is a collection of useful extensions and utilities built around WPILib.
//
//Contents (comment blocks are placed to facilitate searching to find parts of the code quickly):
//	*Math Utilities
//	*Distance Measurement
//	*Controllers/Joysticks
//	*IR Proximity Sensors
//	*Motors
//
//==========================================================================================================================================
#ifndef BOTSMITHS_UTILS_H
#define BOTSMITHS_UTILS_H
#include "WPILib.h"

//Standard library includes likely to be used in robot code
#include <cmath>
#include <string>
#include <sstream>
#include <vector>

#define BOTSMITHS_UTILS_VERSION_MAJOR 1
#define BOTSMITHS_UTILS_VERSION_MINOR 0

//==========================================================================================================================================
//Math Utilities
//==========================================================================================================================================

#define PI 3.14159265f

///@brief Contains useful common math utility functions.
class Math
{
public:

	///@brief Linear interpolation.
	template <typename T>
	static T Lerp(T v0, T v1, T t)
	{
		return (1 - t)*v0 + t*v1;
	}

	///@brief Scales a value to be normalized between 0 and 1 given the range of values it can take. 
	template <typename T>
	static T Normalize(T value, T min, T max)
	{
		return (value - min) / (max - min);
	}
	
	///@brief Clamps a number between a maximum and minimum value.
	template <typename T>
	static T Clamp(T value, T min, T max)
	{
		if ( value < min )
		{
			value = min;
		}
		else if ( value > max )
		{
			value = max;
		}
	
		return value;
	}
};

///@brief 2D vector class for vector math.
template <typename T>
class Vector2T
{
public:

	Vector2T()
		:x(0),
		y(0)
	{}

	Vector2T(T xVal, T yVal)
		:x(xVal),
		y(yVal)
	{}

	Vector2T(const Vector2T& oth)
		:x(oth.x),
		y(oth.y)
	{}

	T x;///< X coordinate of vector
	T y;///< Y coordinate of vector

	///@return The length of this vector. Also referred to as the magnitude of the vector.
	T Length()
	{
		return std::sqrt(x*x + y*y);
	}

	///@return The squared length of this vector. Calculating squared length is faster than calculating length, so if you can use squared length it's better.
	T SquaredLength()
	{
		return x*x + y*y;
	}

	///@brief Normalizes the vector to length 1.0
	Vector2T& Normalize()
	{
		T len = Length();

		x /= len;
		y /= len;

		return *this;
	}

	///@brief Multiply by scalar to scale the vector's length.
	Vector2T operator * (T scalar)
	{
		return Vector2T(x * scalar, y * scalar);
	}

	///@brief Add two vectors
	Vector2T operator + (const Vector2T& oth)
	{
		return Vector2T(x + oth.x, y + oth.y);
	}

	///@brief Subtract two vectors
	Vector2T operator - (const Vector2T& oth)
	{
		return (*this)+((-1)*oth);
	}

	///@brief Converts the vector to a printable string for debugging.
	std::string ToString()
	{
		std::stringstream ss;
		ss << "(" << x << ", " << y << ", " << ")";
		return ss.str();
	}
};

typedef Vector2T<float> Vector2;
typedef Vector2T<double> DoubleVector2;

///@brief 3D vector class for vector math.
template <typename T>
class Vector3T
{
public:

	Vector3T()
		:x(0),
		y(0),
		z(0)
	{}

	Vector3T(T xVal, T yVal, T zVal)
		:x(xVal),
		y(yVal),
		z(zVal)
	{}

	Vector3T(const Vector3T& oth)
		:x(oth.x),
		y(oth.y),
		z(oth.z)
	{}

	T x;///< X coordinate of vector
	T y;///< Y coordinate of vector
	T z;///< Z coordinate of vector

	///@return The length of this vector. Also referred to as the magnitude of the vector.
	T Length()
	{
		return std::sqrt(x*x + y*y + z*z);
	}

	///@return The squared length of this vector. Calculating squared length is faster than calculating length, so if you can use squared length it's better.
	T SquaredLength()
	{
		return x*x + y*y + z*z;
	}

	///@brief Normalizes the vector to length 1.0
	Vector3T& Normalize()
	{
		T len = Length();

		x /= len;
		y /= len;
		z /= len;

		return *this;
	}

	///@brief Multiply by scalar to scale the vector's length.
	Vector3T operator * (T scalar)
	{
		return Vector3T(x * scalar, y * scalar, z * scalar);
	}

	///@brief Add two vectors
	Vector3T operator + (const Vector3T& oth)
	{
		return Vector3T(x + oth.x, y + oth.y, z + oth.z);
	}

	///@brief Subtract two vectors
	Vector3T operator - (const Vector3T& oth)
	{
		return (*this) + ((-1)*oth);
	}

	///@brief Converts the vector to a printable string for debugging.
	std::string ToString()
	{
		std::stringstream ss;
		ss << "(" << x << ", " << y << ", " << z << ")";
		return ss.str();
	}
};

typedef Vector3T<float> Vector3;
typedef Vector3T<double> DoubleVector3;

//==========================================================================================================================================
//Distance Measurement
//==========================================================================================================================================

///@brief The MovementTracker uses the double integration method to track how far the robot has moved using the accelerometer.
///Tracking movement with the accelerometer is not very accurate, so it's important to use it only for moving short distances.
///Resetting the movement tracker once you've reached a known position can be very helpful too.
///The movement tracker is not started automatically, to start tracking movement use the StartTracking method. 
class MovementTracker
{
public:

	MovementTracker(Accelerometer* accel)
		:m_accel(accel),
		m_velocityX(0.0),
		m_positionX(0.0),
		m_velocityY(0.0),
		m_positionY(0.0),
		m_velocityZ(0.0),
		m_positionZ(0.0),
		m_command(NULL)
	{
		
	}

	~MovementTracker()
	{
		StopTracking();
		delete m_command;
	}

	///@brief Starts tracking movement.
	void StartTracking()
	{
		if (!m_command)
		{
			m_command = new TrackerCommand(this);
		}

		m_command->Start();
	}

	///@brief Stops tracking movement.
	///@param resetTracking Whether or not to reset the tracking when it stops.
	void StopTracking(bool resetTracking = false)
	{
		m_command->Cancel();

		if (resetTracking)
		{
			Reset();
		}
	}

	///@brief Resets tracked position and velocity. This should only be used when fully stopped.
	void Reset()
	{
		m_velocityX = 0.0;
		m_positionX = 0.0;
		m_velocityY = 0.0;
		m_positionY = 0.0;
		m_velocityZ = 0.0;
		m_positionZ = 0.0;
	}

	///@return The velocity in the X direction.
	double GetVelocityX()
	{
		return m_velocityX;
	}
	///@return The velocity in the Y direction.
	double GetVelocityY()
	{
		return m_velocityY;
	}
	///@return The velocity in the Z direction.
	double GetVelocityZ()
	{
		return m_velocityZ;
	}

	///@return The change in position from where the robot started on the X axis
	///Axes are defined relative to the robot starting orientation, so the X axis would be to the right of the robot.
	double GetPositionX()
	{
		return m_positionX;
	}
	///@return The change in position from where the robot started on the Y axis
	///Axes are defined relative to the robot starting orientation, so the Y axis would be out the top of the robot.
	///Why you'd ever need to measure this I have no idea... but I put it in there cause why not?
	double GetPositionY()
	{
		return m_positionY;
	}
	///@return The change in position from where the robot started on the Z axis
	///Axes are defined relative to the robot starting orientation, so the Z axis would be to the front of the robot.
	double GetPositionZ()
	{
		return m_positionZ;
	}

private:

	void Update()
	{
		m_velocityX += m_accel->GetX();
		m_positionX += m_velocityX;

		m_velocityY += m_accel->GetY();
		m_positionY += m_velocityY;

		m_velocityZ += m_accel->GetZ();
		m_positionZ += m_velocityZ;
	}

	Accelerometer* m_accel;

	double m_velocityX;///< The current X velocity of the robot calculated from acceleration.
	double m_positionX;///< The current X position of the robot calculated from velocity.

	double m_velocityY;///< The current Y velocity of the robot calculated from acceleration.
	double m_positionY;///< The current Y position of the robot calculated from velocity.

	double m_velocityZ;///< The current Z velocity of the robot calculated from acceleration.
	double m_positionZ;///< The current Z position of the robot calculated from velocity.

	///@brief Command used to schedule updates for movement tracker.
	class TrackerCommand : public Command
	{
	public:

		TrackerCommand(MovementTracker* tracker)
			:m_tracker(tracker)
		{}

		void Initialize()
		{

		}

		void Execute()
		{ 
			m_tracker->Update();
		}

		bool IsFinished()
		{
			return false;
		}

		void End(){}

		void Interrupted(){}

	private:

		MovementTracker* m_tracker;
	};

	TrackerCommand* m_command;
};

//==========================================================================================================================================
//Controllers/Joysticks
//==========================================================================================================================================

///@brief GameController is an interface for using XBox controllers, PlayStation controllers, etc. without having to remember button mappings all the time.
///It serves as a simple wrapper around the Joystick class intended to have button mapping built in.
///@todo Add JoystickButton command creation utility methods.
class GameController : public Joystick
{
	float m_deadZone;

public:

	GameController(uint32_t port, float deadZone)
		:Joystick(port),
		m_deadZone(deadZone)
	{}

	///@return True if the left trigger button on the controller is pressed, false otherwise.
	virtual bool GetLeftTrigger() = 0;
	///@return True if the right trigger button on the controller is pressed, false otherwise.
	virtual bool GetRightTrigger() = 0;

	///@return True if the left bumper on the controller is pressed, false otherwise.
	virtual bool GetLeftBumper() = 0;
	///@return True if the right bumper on the controller is pressed, false otherwise.
	virtual bool GetRightBumper() = 0;
	
	///@return True if the right stick is clicked, false otherwise.
	virtual bool GetRightStickButton() = 0;
	///@return True if the left stick is clicked, false otherwise.
	virtual bool GetLeftStickButton() = 0;
	
	///@return The value of the left stick's X (left/right) axis.
	virtual float GetLeftStickX() = 0;
	///@return The value of the left stick's Y (up/down) axis.
	virtual float GetLeftStickY() = 0;
	///@return The left stick as a normalized vector. This is INCREDIBLY useful for calculating deadzones because you can base the deadzone on the vector length.
	virtual Vector2 GetLeftStick()
	{
		Vector2 stickVal(GetLeftStickX(), GetLeftStickY());

		if (stickVal.Length() <= m_deadZone)
		{
			return Vector2(0.0f, 0.0f);
		}

		stickVal.Normalize();
		return stickVal;
	}

	///@return The value of the right stick's X (left/right) axis.
	virtual float GetRightStickX() = 0;
	///@return The value of the right stick's Y (up/down) axis.
	virtual float GetRightStickY() = 0;
	///@return The right stick as a normalized vector. This is INCREDIBLY useful for calculating deadzones because you can base the deadzone on the vector length.
	virtual Vector2 GetRightStick()
	{
		Vector2 stickVal(GetRightStickX(), GetRightStickY());

		if (stickVal.Length() <= m_deadZone)
		{
			return Vector2(0.0f, 0.0f);
		}

		stickVal.Normalize();
		return stickVal;
	}

	///@return Returns the D-Pad X value (left/right). This will return 0 if the D-Pad is not pressed, and will return positive for right, and negative for left.
	virtual short GetDPadX() = 0;
	///@return Returns the D-Pad Y value (up/down). This will return 0 if the D-Pad is not pressed, and will return positive for up, and negative for down.
	virtual short GetDPadY() = 0;

	///@return True if the A button (Xbox controller layout) is pressed, false otherwise. Equivalent to X button on a PlayStation controller. 
	virtual bool GetAButton() = 0;
	///@return True if the B button (Xbox controller layout) is pressed, false otherwise. Equivalent to Circle button on a PlayStation controller. 
	virtual bool GetBButton() = 0;
	///@return True if the X button (Xbox controller layout) is pressed, false otherwise. Equivalent to Square button on a PlayStation controller. 
	virtual bool GetXButton() = 0;
	///@return True if the Y button (Xbox controller layout) is pressed, false otherwise. Equivalent to Triangle button on a PlayStation controller. 
	virtual bool GetYButton() = 0;

	//Some WPILib Joystick methods will be overriden so that GameController can automatically plug into code written for Joystick with the correct mappings.
	//For example: the Xbox 360 controller has different axes from the Attack 3, and so the code usually has to change in order to switch controller types.
	//Using the Joystick class's GetTrigger() method also ends up mapping to the A button the the Xbox 360 controller.

	virtual float GetX(JoystickHand hand = kRightHand){ return GetLeftStickX(); }
	virtual float GetY(JoystickHand hand = kRightHand){ return GetLeftStickY(); }
	
	//Z and Twist are usually the same thing, so they will both be mapped to the right stick's X axis.
	virtual float GetZ(){ return GetRightStickX(); }
	virtual float GetTwist(){ return GetRightStickX(); }
	//Joystick trigger will be mapped to the right trigger on controllers. 
	virtual bool GetTrigger(JoystickHand hand = kRightHand){ return GetRightTrigger(); }
	//Joystick top button is usually some sort of action, so it gets mapped to X by default.
	virtual bool GetTop(JoystickHand hand = kRightHand){ return GetXButton(); }
};

///@brief Xbox Controller class with axis and button mappings built-in.
class XboxController : public GameController
{
public:

	static const int LEFT_TRIGGER = 2;
	static const int RIGHT_TRIGGER = 3;
	static const int LEFT_BUMPER = 5;
	static const int RIGHT_BUMPER = 6;
	static const int RIGHT_STICK_BUTTON = 10;
	static const int LEFT_STICK_BUTTON = 9;
	static const int LEFT_STICK_X = 0;
	static const int LEFT_STICK_Y = 1;
	static const int RIGHT_STICK_X = 4;
	static const int RIGHT_STICK_Y = 5;
	static const int A_BUTTON = 1;
	static const int B_BUTTON = 2;
	static const int X_BUTTON = 3;
	static const int Y_BUTTON = 4;

	XboxController(uint32_t port, float deadZone = 0.15f)
		:GameController(port, deadZone)
	{}

	virtual bool GetLeftTrigger()
	{
		return GetRawAxis(LEFT_TRIGGER) > 0.5f;
	}
	
	virtual bool GetRightTrigger()
	{
		return GetRawAxis(RIGHT_TRIGGER) > 0.5f;
	}

	virtual bool GetLeftBumper()
	{
		return GetRawButton(LEFT_BUMPER);
	}
	
	virtual bool GetRightBumper()
	{
		return GetRawButton(RIGHT_BUMPER);
	}

	virtual bool GetRightStickButton()
	{
		return GetRawButton(RIGHT_STICK_BUTTON);
	}

	virtual bool GetLeftStickButton()
	{
		return GetRawButton(LEFT_STICK_BUTTON);
	}

	virtual float GetLeftStickX()
	{
		return GetRawAxis(LEFT_STICK_X);
	}

	virtual float GetLeftStickY()
	{
		return GetRawAxis(LEFT_STICK_Y);
	}

	virtual float GetRightStickX()
	{
		return GetRawAxis(RIGHT_STICK_X);
	}
	
	virtual float GetRightStickY()
	{
		return GetRawAxis(RIGHT_STICK_Y);
	}

	virtual short GetDPadX()
	{
		int angle = GetPOV();
		if (angle == 90)
		{
			return 1;
		}
		else if (angle == 270)
		{
			return -1;
		}

		return 0;
	}
	
	virtual short GetDPadY()
	{
		int angle = GetPOV();
		if (angle == 0)
		{
			return 1;
		}
		else if (angle == 180)
		{
			return -1;
		}

		return 0;
	}
	
	virtual bool GetAButton()
	{
		return GetRawButton(A_BUTTON);
	}

	virtual bool GetBButton()
	{
		return GetRawButton(B_BUTTON);
	}

	virtual bool GetXButton()
	{
		return GetRawButton(X_BUTTON);
	}

	virtual bool GetYButton()
	{
		return GetRawButton(Y_BUTTON);
	}
};

///@brief Logitech F310 Controller class with axis and button mappings built-in.
class LogitechF310Controller : public GameController
{
public:

	static const int LEFT_TRIGGER = 3;
	static const int RIGHT_TRIGGER = 3;
	static const int LEFT_BUMPER = 5;
	static const int RIGHT_BUMPER = 6;
	static const int RIGHT_STICK_BUTTON = 10;
	static const int LEFT_STICK_BUTTON = 9;
	static const int LEFT_STICK_X = 1;
	static const int LEFT_STICK_Y = 2;
	static const int RIGHT_STICK_X = 4;
	static const int RIGHT_STICK_Y = 5;
	static const int A_BUTTON = 1;
	static const int B_BUTTON = 2;
	static const int X_BUTTON = 3;
	static const int Y_BUTTON = 4;

	LogitechF310Controller(uint32_t port, float deadZone = 0.05f)
		:GameController(port, deadZone)
	{}

	virtual bool GetLeftTrigger()
	{
		return GetRawAxis(LEFT_TRIGGER) > 0.5f;
	}

	virtual bool GetRightTrigger()
	{
		return GetRawAxis(RIGHT_TRIGGER) < -0.5f;
	}

	virtual bool GetLeftBumper()
	{
		return GetRawButton(LEFT_BUMPER);
	}

	virtual bool GetRightBumper()
	{
		return GetRawButton(RIGHT_BUMPER);
	}

	virtual bool GetRightStickButton()
	{
		return GetRawButton(RIGHT_STICK_BUTTON);
	}

	virtual bool GetLeftStickButton()
	{
		return GetRawButton(LEFT_STICK_BUTTON);
	}

	virtual float GetLeftStickX()
	{
		return GetRawAxis(LEFT_STICK_X);
	}

	virtual float GetLeftStickY()
	{
		return GetRawAxis(LEFT_STICK_Y);
	}

	virtual float GetRightStickX()
	{
		return GetRawAxis(RIGHT_STICK_X);
	}

	virtual float GetRightStickY()
	{
		return GetRawAxis(RIGHT_STICK_Y);
	}

	virtual short GetDPadX()
	{
		int angle = GetPOV();
		if (angle == 90)
		{
			return 1;
		}
		else if (angle == 270)
		{
			return -1;
		}

		return 0;
	}

	virtual short GetDPadY()
	{
		int angle = GetPOV();
		if (angle == 0)
		{
			return 1;
		}
		else if (angle == 180)
		{
			return -1;
		}

		return 0;
	}

	virtual bool GetAButton()
	{
		return GetRawButton(A_BUTTON);
	}

	virtual bool GetBButton()
	{
		return GetRawButton(B_BUTTON);
	}

	virtual bool GetXButton()
	{
		return GetRawButton(X_BUTTON);
	}

	virtual bool GetYButton()
	{
		return GetRawButton(Y_BUTTON);
	}
};

///@brief Wraps options for controllers, such as whether or not the Y axis should be inverted, axis sensitivity, etc.
class ControllerOptions
{
public:

	///@return Whether or not the controller in the assigned port should have its Y axis inverted.
	static bool InvertY(unsigned int controllerNum)
	{
		char controllerString[255];
		sprintf(controllerString, "Controller %d Invert Y", controllerNum);

		if (!Preferences::GetInstance()->Contains(controllerString))
		{
			Preferences::GetInstance()->PutBoolean(controllerString, false);
		}

		return Preferences::GetInstance()->GetBoolean(controllerString);
	}

	///@return The sensitivity value for the controller at the specified port.
	static float Sensitivity(unsigned int controllerNum)
	{
		char controllerString[255];
		sprintf(controllerString, "Controller %d Sensitivity", controllerNum);

		if (!Preferences::GetInstance()->Contains(controllerString))
		{
			Preferences::GetInstance()->PutFloat(controllerString, 1.0f);
		}

		return Preferences::GetInstance()->GetFloat(controllerString, 1.0f);
	}
};

//==========================================================================================================================================
//IR Proximity Sensors
//==========================================================================================================================================

///@brief Proximity sensor interface for Sharp infrared proximity sensors.
/*class ProximitySensor : public PIDSource
{
public:
	
	///@return The maximum distance that this sensor can read (in cm).
	virtual double GetMaxRange()=0;
	///@return The minimum distance that this sensor can read (in cm).
	virtual double GetMinRange()=0;
	
	///@return Returns the current distance in front of the proximity sensor (in cm).
	virtual double GetDistance()=0;
	
	virtual double PIDGet(){return GetDistance();}
};
*/
///@brief Infrared Proximity Sensor - Sharp GP2Y0A21YK 
///https://www.sparkfun.com/products/242
template <int LOOKUP_TABLE_SIZE>
class ProximitySensor : public PIDSource
{
public:
	
	ProximitySensor(unsigned int analogChannel)
		:m_analogInput(analogChannel),
		m_maxVoltage(3.1f),
		m_minVoltage(0.3f)
	{
		
	}

	virtual ~ProximitySensor(){}
	
	///@return The distance in front of the range finder.
	float GetDistance()
	{
		float voltage = m_analogInput.GetVoltage();
		voltage = Math::Normalize(voltage, m_minVoltage, m_maxVoltage);//Normalize the voltage to a usable value.
		
		int lookupIdxLow = std::floor(voltage*(float)LOOKUP_TABLE_SIZE);
		int lookupIdxHigh = std::ceil(voltage*(float)LOOKUP_TABLE_SIZE);
		
		lookupIdxLow = Math::Clamp(lookupIdxLow, 0, LOOKUP_TABLE_SIZE);
		lookupIdxHigh = Math::Clamp(lookupIdxHigh, 0, LOOKUP_TABLE_SIZE);
		
		float resultLow = m_rangeLookup(lookupIdxLow);
		float resultHigh = m_rangeLookup(lookupIdxHigh);
		
		//Interpolate between underestimate and overestimate
		float result = Math::Lerp(resultLow, resultHigh, (voltage*((float)LOOKUP_TABLE_SIZE-(float)lookupIdxLow)));
		
		return result;
	}
	
protected:
	
	float m_rangeLookup[LOOKUP_TABLE_SIZE];
	
	AnalogInput m_analogInput;
	
	float m_maxVoltage;
	float m_minVoltage;
};

//==========================================================================================================================================
//Motors
//==========================================================================================================================================

template <typename T>
class SpeedControllerRamp : public SpeedController
{
public:

	SpeedControllerRamp(uint32_t channel, float accelerationDamping=4.0f)
		:m_motorController(channel),
		m_targetSpeed(0.0f),
		m_currentSpeed(0.0f),
		m_accelerationDamping(accelerationDamping)
	{
		m_command = new SpeedControllerRampCommand(this);
		m_command->Start();
	}

	virtual ~SpeedControllerRamp()
	{
		m_command->Cancel();
		delete m_command;
	}

	virtual void Set(float value, uint8_t syncGroup = 0)
	{
		m_targetSpeed = value;
	}

	virtual float Get()
	{
		return m_motorController.Get();
	}

	virtual void Disable()
	{
		m_motorController.Disable();
	}

	virtual void PIDWrite(float output)
	{
		m_motorController.PIDWrite(output);
	}

	///@brief Updates the motor speed values. This will be called automatically if the Scheduler is being used (used for all command based robots).
	///If the Scheduler is not being used, then each motor must be updated individually.
	void Update()
	{
		m_currentSpeed = Math::Lerp(m_currentSpeed, m_targetSpeed, 0.06f*m_accelerationDamping);
		m_motorController.Set(m_currentSpeed);
	}

private:

	float m_targetSpeed;
	float m_currentSpeed;
	float m_accelerationDamping;

	class SpeedControllerRampCommand : public Command
	{
	public:

		SpeedControllerRampCommand(SpeedControllerRamp* controllerRamp)
			:m_controllerRamp(controllerRamp)
		{}

		void Initialize(){}

		void Execute()
		{
			m_controllerRamp->Update();
		}

		bool IsFinished(){return false;}

		void End(){}

		void Interrupted(){}

	private:

		SpeedControllerRamp* m_controllerRamp;
	};

	SpeedControllerRampCommand* m_command;

	T m_motorController;
};

#endif