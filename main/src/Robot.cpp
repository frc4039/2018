#include <iostream>
#include <memory>
#include <string>
#include <ctime>
#include <fstream>
#include <thread>
#include <list>
#include <mutex>
#include <sstream>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include "WPILib.h"
#include "SimPID.h"
#include "AHRS.h"
#include <IterativeRobot.h>
#include <LiveWindow/LiveWindow.h>
#include "shiftlib.h"
#include "ctre/Phoenix.h"

#define GP_L 5
#define GP_R 6

//#define AUX_PWM

class Robot: public frc::IterativeRobot {
public:

	Joystick *m_Joystick; // The place where you define your variables
	Joystick *m_Joystick2;

	VictorSP *m_LFMotor; // ^
	VictorSP *m_LBMotor; // ^
	VictorSP *m_RFMotor; // ^
	VictorSP *m_RBMotor; // ^
	VictorSP *m_lowerIntakeL; // left lower intake wheel
	VictorSP *m_lowerIntakeR; // right lower intake wheel
	VictorSP *m_conveyor;

	Solenoid *m_shiftHigh;
	Solenoid *m_shiftLow;
//	int driveState;
	int autoState, autoMode, autoDelay;
	//Talon

#ifdef AUX_PWM
	VictorSP *m_elevatorPWM;
#else
	TalonSRX *m_elevatorCAN;
#endif

	XboxController *m_Gamepad;

	Solenoid *m_testExtend;
	Solenoid *m_testRetract;

	Encoder *m_leftEncoder;
	Encoder *m_rightEncoder;

	SimPID *m_drivePID;
	SimPID *m_turnPID;
	SimPID *m_finalTurnPID;

	Path *p_step_one;
	Path *p_step_two;
	Path *p_step_three;

	//====================Pathfollow Variables==================
	PathFollower *BBYCAKES;

	//drive to switch, drop cube on ends
	Path *path_centreSwitchLeft, *path_centreSwitchRight;
	//drive to swtich, drop cube via on close face
	Path *path_centreSwitchLeft2, *path_centreSwitchRight2, *path_backupLeft, *path_backupRight;
	//drive to switch and stop from any station
	Path *path_sideVeerLeft, *path_sideVeerRight, *path_sideCrossLeft, *path_sideCrossRight;
	//drive straight, drop cube if your side
	Path *path_, *path_autoThreeSwitchStraight3;

	AHRS *nav;

	std::string plateColour;

	Timer *autoTimer;
	Timer *delayTimer;

/*	static void VisionThread()
    {
        cs::UsbCamera camera = CameraServer::GetInstance()->StartAutomaticCapture();
        camera.SetResolution(640, 480);
        cs::CvSink cvSink = CameraServer::GetInstance()->GetVideo();
        cs::CvSource outputStreamStd = CameraServer::GetInstance()->PutVideo("Gray", 640, 480);
        cv::Mat source;
        cv::Mat output;
        while(true) {
            cvSink.GrabFrame(source);
            cvtColor(source, output, cv::COLOR_BGR2GRAY);
            outputStreamStd.PutFrame(output);
        }
    }*/

	void RobotInit() {
		m_LFMotor = new VictorSP(6); // The place where you initialize your variables to their victors and which port on the RoboRio/computer
		m_LFMotor->SetSafetyEnabled(true);
		m_LBMotor = new VictorSP(7); // ^
		m_LBMotor->SetSafetyEnabled(true);
		m_RFMotor = new VictorSP(8); // ^
		m_RFMotor->SetSafetyEnabled(true);
		m_RBMotor = new VictorSP(9); // ^
		m_RBMotor->SetSafetyEnabled(true);

		CameraServer::GetInstance()->StartAutomaticCapture();

		m_lowerIntakeL = new VictorSP(1); // left intake
		m_lowerIntakeR = new VictorSP(2); // right intake
		m_conveyor = new VictorSP(0);

		m_Joystick = new Joystick(0); // ^
		m_Joystick2 = new Joystick(1);// ^

		m_Gamepad = new XboxController(2);

		m_shiftHigh = new Solenoid(0);
		m_shiftLow = new Solenoid(1);
		m_testExtend = new Solenoid(2);
		m_testRetract = new Solenoid(3);

#ifdef AUX_PWM
		m_elevatorPWM = new VictorSP(6);
#else
		m_elevatorCAN = new TalonSRX(1);
		m_elevatorCAN->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative, 0, 1000);
#endif

		autoTimer = new Timer();
		autoTimer->Reset();
		autoTimer->Stop();

		delayTimer = new Timer();
		delayTimer->Reset();
		delayTimer->Stop();

		//================Define Auto Paths===============
		int zero[2] = {0, 0};

		int centreLeftEnd[2] = {6400, -7000};
		int cp1[2] = {0, -7000};
		int cp2[2] = {6400, 0};
		path_centreSwitchLeft = new PathCurve(zero, cp1, cp2, centreLeftEnd, 40);

		int centreRightEnd[2] = {6400, 6400};
		int cp3[2] = {0, 6400};
		int cp4[2] = {6400, 0};
		path_centreSwitchRight = new PathCurve(zero, cp3, cp4, centreRightEnd, 40);

		int centreLeftEnd2[2] = {5400, -6000};
		int cp5[2] = {1000, 0};
		int cp6[2] = {4400, -6000};
		path_centreSwitchLeft2 = new PathCurve(zero, cp5, cp6, centreLeftEnd2, 40);
		int cp7[2] = {4400, -7600};
		path_backupLeft = new PathCurve(centreLeftEnd2, cp6, cp7, centreLeftEnd, 40);

		int centreRightEnd2[2] = {5400, 5400};
		int cp8[2] = {1000, 0};
		int cp9[2] = {4400, 5400};
		path_centreSwitchRight2 = new PathCurve(zero, cp8, cp9, centreRightEnd2, 40);
		int cp10[2] = {4400, -7000};
		path_backupRight = new PathCurve(centreRightEnd2, cp9, cp10, centreRightEnd, 40);

		int sideVLEnd[2] = {6400, -800};
		int cp11[2] = {2000, 800};
		int cp12[2] = {7000, 0};
		path_sideVeerLeft = new PathCurve(zero, cp11, cp12, sideVLEnd, 40);

		int sideVREnd[2] = {6400, 800};
		int cp13[2] = {2000, -800};
		path_sideVeerRight = new PathCurve(zero, cp13, cp12, sideVREnd, 40);

		nav = new AHRS(SPI::Port::kMXP);
		nav->Reset();

/*		std::thread visionThread(VisionThread);
		visionThread.detach();*/

//		driveState = 7;
		autoMode = 0;
		autoState = 0;
		autoDelay = 0;

		m_leftEncoder = new Encoder(2,3);
		m_rightEncoder = new Encoder(0,1);

		m_turnPID = new SimPID(1.0, 0, 0.02, 0, 5.0);
		m_turnPID->setContinuousAngle(true);
		m_drivePID = new SimPID(0.001, 0, 0.002, 0, 200);
		m_drivePID->setMaxOutput(0.9);
		m_finalTurnPID = new SimPID(0.9, 0, 0.02, 0, 5.0);
		m_finalTurnPID->setContinuousAngle(true);

		BBYCAKES = new PathFollower(500, PI/3, m_drivePID, m_turnPID, m_finalTurnPID);
		BBYCAKES->setIsDegrees(true);

		int step_one[2] = {-5000, 0};
		int step_two[2] = {0, -2887};
		int end[2] = {-5000, -2887};

		p_step_one = new PathLine(zero, step_one, 10);
		p_step_two = new PathLine(step_one, step_two, 10);
		p_step_three = new PathLine(step_two, end, 10);
	}

	void DisabledPeriodic() {
		long long int LRead = m_leftEncoder->Get();
		long long int RRead = m_rightEncoder->Get();
		long long int ERead = m_elevatorCAN->GetSelectedSensorPosition(0);
		long double GRead = nav->GetYaw();
		for(int i = 1; i <= 12; i++) {
			if(m_Joystick->GetRawButton(i))	{
				autoMode = i;
				nav->Reset();
				m_leftEncoder->Reset();
				m_rightEncoder->Reset();
				m_elevatorCAN->SetSelectedSensorPosition(0, 0, 10);
			}
		}
		autoDelay = m_Joystick->GetRawAxis()
		DriverStation::ReportError("Left Encoder: " + std::to_string(LRead) + " | Right Encoder: " + std::to_string(RRead) + " | Gyro: " + std::to_string(GRead));
		DriverStation::ReportError("Auto Mode: " + std::to_string(autoMode) + " | Elevator Pos: " + std::to_string(ERead));
	}

	void AutonomousInit() override {
		m_LFMotor->SetSpeed(0.f);
		m_LBMotor->SetSpeed(0.f);
		m_RFMotor->SetSpeed(0.f);
		m_RBMotor->SetSpeed(0.f);
		m_leftEncoder->Reset();
		m_rightEncoder->Reset();
//		nav->Reset();
		autoState = 0;
		autoTimer->Reset();
		autoTimer->Start();
		delayTimer->Reset();
		delayTimer->Start();
		plateColour = DriverStation::GetInstance().GetGameSpecificMessage();
	}

	void AutonomousPeriodic() {
		if(delayTimer->Get() > autoDelay) {
			switch(autoMode) {
			case 1: //deploy cubes on sides of switch
				switch(plateColour[0]) {
				case 'L':
					switch(autoState) {
					case 0:
						BBYCAKES->initPath(path_centreSwitchLeft, PathForward, 90);
						autoState++;
						break;
					case 1:
						if(advancedAutoDrive())
							autoState++;
						break;
					case 2:
						m_lowerIntakeL->SetSpeed(-1.f);
						m_lowerIntakeR->SetSpeed(1.f);
						break;
					}
					break;
				case 'R':
					switch(autoState) {
					case 0:
						BBYCAKES->initPath(path_centreSwitchRight, PathForward, -90);
						autoState++;
						break;
					case 1:
						if(advancedAutoDrive())
							autoState++;
						break;
					case 2:
						m_lowerIntakeL->SetSpeed(-1.f);
						m_lowerIntakeR->SetSpeed(1.f);
						break;
					}
					break;
				}
				break;
			case 2: //deploy cube on close face of the switch
				switch(plateColour[0]) {
				case 'L':
					switch(autoState) {
					case 0:
						BBYCAKES->initPath(path_centreSwitchLeft2, PathForward, 0);
						autoState++;
						break;
					case 1:
						if(advancedAutoDrive()) {
							autoState++;
							autoTimer->Reset();
						}
						break;
					case 2:
						m_lowerIntakeL->SetSpeed(-1.f);
						m_lowerIntakeR->SetSpeed(1.f);
						BBYCAKES->initPath(path_backupLeft, PathBackward, 180);
						if(autoTimer->Get() > 2.5)
							autoState++;
						break;
					case 3:
						advancedAutoDrive();
						break;
					}
					break;
				case 'R':
					switch(autoState) {
					case 0:
						BBYCAKES->initPath(path_centreSwitchRight2, PathForward, 0);
						autoState++;
						break;
					case 1:
						if(advancedAutoDrive()) {
							autoState++;
							autoTimer->Reset();
						}
						break;
					case 2:
						m_lowerIntakeL->SetSpeed(-1.f);
						m_lowerIntakeR->SetSpeed(1.f);
						BBYCAKES->initPath(path_backupRight, PathBackward, 180);
						if(autoTimer->Get() > 2.5)
							autoState++;
						break;
					case 3:
						advancedAutoDrive();
						break;
					}
					break;
				}
				break;
			case 3: //
				switch(autoState) {
				case 0:
					BBYCAKES->initPath(path_sideVeerLeft, PathForward, -90);
				}
			}
		}
	}

	void TeleopInit() {
		m_LFMotor->SetSpeed(0.f);
		m_LBMotor->SetSpeed(0.f);
		m_RFMotor->SetSpeed(0.f);
		m_RBMotor->SetSpeed(0.f);
		m_lowerIntakeL->SetSpeed(0.f);
		m_lowerIntakeR->SetSpeed(0.f);
	}

	void TeleopPeriodic() {
		driveShift();
	//	simpleElev();
		advancedElev();
//		switch(driveState) {
//		case 7:
		arcadeDrive();
/*			break;
		case 8:
			tankDrive();
			break;
		case 9:
			cheezyDrive();
			break;
		}*/
		simpleIntake();
	}

	void driveShift() {
		if(m_Joystick->GetRawButton(1)) {
			m_shiftHigh->Set(true);
			m_shiftLow->Set(false);
		}
		else {
			m_shiftHigh->Set(false);
			m_shiftLow->Set(true);
		}
	}

	void simpleElev() {
		float elevSpeed = 0.65*limit(m_Gamepad->GetRawAxis(1));
#ifdef AUX_PWM
		m_elevatorPWM->Set(limit(-elevSpeed));
#else
		m_elevatorCAN->Set(ControlMode::PercentOutput, limit(-elevSpeed + 0.1));
#endif
	}

	void advancedElev() {
		if(m_Joystick->GetRawButton(12))
			m_elevatorCAN->Set(ControlMode::Position, -3000);
		else if(m_Joystick->GetRawButton(11))
			m_elevatorCAN->Set(ControlMode::Position, 0);
	}

	bool advancedAutoDrive() {
		float leftSpeed, rightSpeed;
		if(BBYCAKES->followPathByEnc(m_leftEncoder->Get(), m_rightEncoder->Get(), nav->GetYaw(), leftSpeed, rightSpeed) == 0){
			m_LFMotor->SetSpeed(leftSpeed);
			m_LBMotor->SetSpeed(leftSpeed);
			m_RFMotor->SetSpeed(rightSpeed);
			m_RBMotor->SetSpeed(rightSpeed);
		}
		printf("path follow left: %f, right: %f\n", leftSpeed, rightSpeed);
		return BBYCAKES->isDone();
	}

	void arcadeDrive() {
		//R = +y - x;
		//L = -y - x;
		float joyX;
		float joyY;

		joyX = -limit(expo(m_Joystick->GetX(), 3)); // Getting the X position from the joystick
		joyY = -limit(expo(m_Joystick->GetY(), 2)); // Getting the Y position from the joystick

		m_LFMotor->SetSpeed(-joyY + joyX);
		m_LBMotor->SetSpeed(-joyY + joyX);
		m_RFMotor->SetSpeed(joyY + joyX);
		m_RBMotor->SetSpeed(joyY + joyX);
	}

	void tankDrive() {
		float joy1Y;
		float joy2Y;

		joy1Y = -limit(expo(m_Joystick->GetY(), 2));
		joy2Y = -limit(expo(m_Joystick2->GetY(), 2));

		m_LFMotor->SetSpeed(-joy1Y);
		m_LBMotor->SetSpeed(-joy1Y);
		m_RFMotor->SetSpeed(joy2Y);
		m_RBMotor->SetSpeed(joy2Y);
	}

	void simpleIntake() {
		float intakeFSpeed = m_Gamepad->GetBumper(XboxController::kLeftHand);
		float intakeRSpeed = m_Gamepad->GetBumper(XboxController::kRightHand);

		m_lowerIntakeL->SetSpeed(intakeFSpeed - intakeRSpeed);
		m_lowerIntakeR->SetSpeed(-intakeFSpeed + intakeRSpeed);
	}

	void cheezyDrive() {
		float joy1Y;
		float joy2X;

		joy1Y = -limit(expo(m_Joystick->GetY(), 2));
		joy2X = -limit(expo(m_Joystick2->GetX(), 3));

		m_LFMotor->SetSpeed(-joy1Y - joy2X);
		m_LBMotor->SetSpeed(-joy1Y - joy2X);
		m_RFMotor->SetSpeed(joy1Y - joy2X);
		m_RBMotor->SetSpeed(joy1Y - joy2X);

	}

// #.f is used to tell the computer that the number here is not a whole number(1).
	float limit(float n) { // The range and domain are equal to 1.f and -1.f
		if(n>1.f)
			return 1.f; // If "n" is greater then 1.f then return positive 1.f
		if(n<-1.f)
			return -1.f; // If "n" is less then -1.f then return negative -1.f
		return n;
	}

	float expo(float b, int x) {
		float r = 1;
		if(x % 2 == 0) {
			for(int i = 1; i <= x; i++)
				r *= b;
			if(b >= 0)
				return r;
			else
				return -r;
		}
		else {
			for(int i = 1; i <= x; i++)
				r *= b;
			return r;
		}
	}

//	template <typename T>
/*	std::string to_string(const T& value) {
	    std::stringstream ss;
	    ss << value;
	    return ss.str();
	}*/

	void TestPeriodic() {

	}

private:

};

START_ROBOT_CLASS(Robot)
