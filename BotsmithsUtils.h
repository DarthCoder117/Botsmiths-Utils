b//==========================================================================================================================================
//Botsmiths Single Header Utility Library (tentative title while I think of a cool acronym)
//
//The Botsmiths Single Header Utility Library is a collection of useful extensions and utilities built around WPILib.
//
//Contents (comment blocks are placed to facilitate searching to find parts of the code quickly):
//	*Distance Measurement
//	*Controllers/Joysticks
//
//==========================================================================================================================================
#ifndef BOTSMITHS_UTILS_H
#define BOTSMITHS_UTILS_H
#include "WPILib.h"

#define BOTSMITHS_UTILS_VERSION_MAJOR 1
#define BOTSMITHS_UTILS_VERSION_MINOR 0

//==========================================================================================================================================
//Distance Measurement
//==========================================================================================================================================

///@brief The MovementTracker uses the double integration method to track how far the robot has moved using the accelerometer.
///Tracking movement with the accelerometer is not very accurate, so it's important to use it only for moving short distances.
///Resetting the movement tracker once you've reached a known position can be very helpful too.
class MovementTracker
{
public:

	MovementTracker(Accelerometer* accel)
		:m_accel(accel),
		m_velocityX(0.0),
		m_distanceX(0.0),
		m_velocityY(0.0),
		m_distanceY(0.0),
		m_velocityZ(0.0),
		m_distanceZ(0.0)
	{
		m_command = new TrackerCommand(this);
		m_command->Start();
	}

	~MovementTracker()
	{
		m_command->Cancel();
		delete m_command;
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
public:

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

	///@return The value of the right stick's X (left/right) axis.
	virtual float GetRightStickX() = 0;
	///@return The value of the right stick's Y (up/down) axis.
	virtual float GetRightStickY() = 0;

	///@return Returns the D-Pad X value (left/right). This will return 0 if the D-Pad is not pressed, and will return positive for right, and negative for left.
	virtual short GetDPadX() = 0;
	///@return Returns the D-Pad Y value (up/down). This will return 0 if the D-Pad is not pressed, and will return positive for up, and negative for down.
	virtual short GetDPadY() = 0;

	///@return True if the A button (Xbox controller layout) is pressed, false otherwise. Equivalent to X button on a PlayStation controller. 
	virtual bool GetAButton() = 0;
	///@return True if the B button (Xbox controller layout) is pressed, false otherwise. Equivalent to Circle button on a PlayStation controller. 
	virtual bool GetBButton() = 0;
	///@return True if the X button (Xbox controller layout) is pressed, false otherwise. Equivalent to Square button on a PlayStation controller. 
	virtual bool GetXButton() = 0
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
	static const int LEFT_BUMPER = 4;
	static const int RIGHT_BUMPER = 5;
	static const int RIGHT_STICK_BUTTON = 9;
	static const int LEFT_STICK_BUTTON = 8;
	static const int LEFT_STICK_X = 0;
	static const int LEFT_STICK_Y = 1;
	static const int RIGHT_STICK_X = 4;
	static const int RIGHT_STICK_Y = 5;
	static const int A_BUTTON = 0;
	static const int B_BUTTON = 1;
	static const int X_BUTTON = 2;
	static const int Y_BUTTON = 3;

	XboxController(uint32_t port)
		:Joystick(port)
	{}

	XboxController(uint32_t port, uint32_t numAxisTypes, uint32_t numButtonTypes)
		:Joystick(port, numAxisTypes, numButtonTypes)
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

	LogitechF310Controller(uint32_t port)
		:Joystick(port)
	{}

	LogitechF310Controller(uint32_t port, uint32_t numAxisTypes, uint32_t numButtonTypes)
		:Joystick(port, numAxisTypes, numButtonTypes)
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

///@brief Interface for creating joysticks of different types.
class JoystickCreator : public NamedSendable
{
public:

	virtual std::string GetName() = 0;

	///@brief Creates the joystick type that this JoystickCreator is setup for.
	virtual Joystick* CreateJoystick(unsigned int portNumber) = 0;
};

///@brief Template implementation of JoystickCreator for creating Joystick objects of whatever type.
template <typename T>
class JoystickTypeCreator : public IJoystickCreator
{
public:

	virtual std::string GetName()
	{
		return sd::string(typeid(T).name);
	}

	virtual Joystick* CreateJoystick(unsigned int portNumber)
	{
		return new T(portNumber);
	}
};

///@brief Allows selection of different joystick types.
class JoystickSelector
{
public:

	///@brief Creates a joystick selection SendableChooser that can be used to choose what kind of Joystick will be used.
	template <int N>
	static SendableChooser* create(const std::string& name, JoystickCreator* factories[N])
	{
		SendableChooser* chooser = new SendableChooser();
		if (N > 0)
		{
			chooser->AddDefault(factories[0]->GetName(), factories[0]);
		}

		//The default has already been added from index 0, so fill the array with the rest of them.
		for (int i = 1; i < N; ++i)
		{
			chooser->AddObject(factories[i]->GetName(), factories[i]);
		}

		SmartDashboard::PutData(name, chooser);
		return chooser;
	}
};

///@brief Wraps options for controllers, such as whether or not the Y axis should be inverted, axis sensitivity, etc.
class ControllerOptions
{
public:

	///@return Whether or not the controller in the assigned port should have its Y axis inverted.
	static bool InvertY(unsigned int controllerNum)
	{
		return Preferences::GetInstance()->GetBoolean("Controller " + itoa(controllerNum) + "Invert Y");
	}

	///@return The sensitivity value for the controller at the specified port.
	static float Sensitivity(unsigned int controllerNum)
	{
		return Preferences::GetInstance()->GetFloat("Controller " + itoa(controllerNum) + "Sensitivity", 1.0f);
	}
};

#endif