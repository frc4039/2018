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

	//declare class members/variables

	Joystick *m_Joystick;
	Joystick *m_Joystick2;

	VictorSP *m_LFMotor, *m_LBMotor, *m_RFMotor, *m_RBMotor;
	VictorSP *m_lowerIntakeL, *m_lowerIntakeR;
	VictorSP *m_conveyor;

	TalonSRX *m_upperIntakeL, *m_upperIntakeR;
	TalonSRX *m_climber1, *m_climber2;

	Solenoid *m_shiftLow;
	Solenoid *m_shiftHigh;

	int autoState, autoMode, autoDelay, driveState, cheezyState, lowerIntakeState;
	bool shiftToggleState1, shiftToggleState2, intakeToggleState1, intakeToggleState2, autoRunTwelve;

	XboxController *m_GamepadOp;
	XboxController *m_GamepadDr;

	Solenoid *m_gripperExtend;
	Solenoid *m_gripperRetract;
	Solenoid *m_gripperUp;
	Solenoid *m_gripperDown;

	Encoder *m_leftEncoder;
	Encoder *m_rightEncoder;

	SimPID *m_drivePID;
	SimPID *m_turnPID;
	SimPID *m_finalTurnPID;

	Path *p_step_one;
	Path *p_step_two;
	Path *p_step_three;

	AHRS *nav;

	std::string plateColour;

	Timer *autoTimer;
	Timer *delayTimer;
	Timer *triggerTimer;
	Timer *intakeTimer;

	Servo *m_tailgateServo;

	//====================Pathfollow Variables==================
	PathFollower *BBYCAKES;

	//drive to switch, drop cube on ends
	Path *path_centreSwitchLeft, *path_centreSwitchRight;
	//drive to swtich, drop cube via on close face
	Path *path_centreSwitchLeft2, *path_centreSwitchRight2, *path_backupLeft, *path_backupRight;
	//drive to switch and stop from any station
	Path *path_sideVeerLeft, *path_sideVeerRight, *path_sideCrossLeft, *path_sideCrossRight;
	//drive past auto line
	Path *path_crossAutoLine;
	//exchange
	Path *path_exchange;

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
		m_upperIntakeL = new TalonSRX(1);
		m_upperIntakeR = new TalonSRX(2);

		m_climber1 = new TalonSRX(3);
		m_climber2 = new TalonSRX(4);

		m_Joystick = new Joystick(0); // ^
		m_Joystick2 = new Joystick(1);// ^

		m_GamepadOp = new XboxController(2);
		m_GamepadDr = new XboxController(3);

		m_shiftLow = new Solenoid(0);
		m_shiftHigh = new Solenoid(1);
		m_gripperExtend = new Solenoid(2);
		m_gripperRetract = new Solenoid(3);
		m_gripperUp = new Solenoid(4);
		m_gripperDown = new Solenoid(5);

		m_tailgateServo = new Servo(4);

		autoTimer = new Timer();
		autoTimer->Reset();
		autoTimer->Stop();

		delayTimer = new Timer();
		delayTimer->Reset();
		delayTimer->Stop();

		triggerTimer = new Timer();
		triggerTimer->Reset();
		triggerTimer->Stop();

		intakeTimer = new Timer();
		intakeTimer->Reset();
		intakeTimer->Stop();

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

		int sideCLEnd[2] = {6400, -15000};
		int cp14[2] = {7500, 800};
		int cp15[2] = {7400, -16000};
		path_sideCrossLeft = new PathCurve(zero, cp14, cp15, sideCLEnd, 40);

		int sideCREnd[2] = {6400, -15000};
		int cp16[2] = {7500, -800};
		int cp17[2] = {7400, 16000};
		path_sideCrossRight = new PathCurve(zero, cp16, cp17, sideCREnd, 40);

		int crossAutoEnd[2] = {5400, 0};
		path_crossAutoLine = new PathLine(zero, crossAutoEnd, 10);

		int exchangeEnd[2] = {0, -800};
		int cp18[2] = {800, 0};
		int cp19[2] = {800, -800};
		path_exchange = new PathCurve(zero, cp18, cp19, exchangeEnd, 40);

		nav = new AHRS(SPI::Port::kMXP);
		nav->Reset();

/*		std::thread visionThread(VisionThread);
		visionThread.detach();*/

		driveState = 2;
		cheezyState = 1;
		lowerIntakeState = 0;
		shiftToggleState1 = false;
		shiftToggleState2 = false;
		autoRunTwelve = false;
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
	}

	void DisabledPeriodic() {
		long long int LRead = m_leftEncoder->Get();
		long long int RRead = m_rightEncoder->Get();
		long double GRead = nav->GetYaw();
		for(int i = 1; i <= 12; i++) {
			if(m_Joystick->GetRawButton(i))	{
				autoMode = i;
				nav->Reset();
				m_leftEncoder->Reset();
				m_rightEncoder->Reset();
				triggerTimer->Start();
				if(triggerTimer->Get() > 4.f)
					driveState = 1;
			}
			else {
				triggerTimer->Reset();
				triggerTimer->Stop();
			}
		}
		for(int i = 1; i <= 2; i++) {
			if(m_GamepadDr->GetRawButton(i))
				cheezyState = i;
		}
		if(m_GamepadDr->GetBumper(XboxController::kLeftHand) && m_GamepadDr->GetBumper(XboxController::kRightHand)) {
			driveState = 2;
		}
		autoDelay = m_Joystick->GetRawAxis(4);
		DriverStation::ReportError("Left Encoder: " + std::to_string(LRead) + " | Right Encoder: " + std::to_string(RRead) + " | Gyro: " + std::to_string(GRead));
		DriverStation::ReportError("Auto Mode: " + std::to_string(autoMode) + " | Auto Delay: " + std::to_string(autoDelay));
		DriverStation::ReportError("Drive State: " + std::to_string(driveState) + " | Cheezy State: " + std::to_string(cheezyState));
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
		autoRunTwelve = false;
		autoTimer->Reset();
		autoTimer->Start();
		delayTimer->Reset();
		delayTimer->Start();
		plateColour = DriverStation::GetInstance().GetGameSpecificMessage();
	}

	void AutonomousPeriodic() {
		if(delayTimer->Get() > autoDelay) {
			switch(autoMode) {
			case 1: //start from centre, deploy cubes on sides of switch
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
						m_conveyor->Set(1.f);
						m_upperIntakeL->Set(ControlMode::PercentOutput, -1.f);
						m_upperIntakeR->Set(ControlMode::PercentOutput, 1.f);
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
						m_conveyor->Set(1.f);
						m_upperIntakeL->Set(ControlMode::PercentOutput, -1.f);
						m_upperIntakeR->Set(ControlMode::PercentOutput, 1.f);
						break;
					}
					break;
				}
				switch(autoState) {
				case 3:
					dyrdyr
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
						BBYCAKES->initPath(path_backupRight, PathBackward, -180);
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
			case 3: //drive forward from right side and veer left, deploy cube if corresponding side
				switch(autoState) {
				case 0:
					BBYCAKES->initPath(path_sideVeerLeft, PathForward, -90);
					autoState++;
					break;
				case 1:
					if(advancedAutoDrive()) {
						if(plateColour[0] == 'R')
							autoState++;
					}
					break;
				case 2:
					m_conveyor->SetSpeed(1.f);
					m_upperIntakeL->Set(ControlMode::PercentOutput, -1.f);
					m_upperIntakeR->Set(ControlMode::PercentOutput, 1.f);
					break;
				}
				break;
			case 4: //drive forward from left side and veer right, deploy cube if corresponding side
				switch(autoState) {
				case 0:
					BBYCAKES->initPath(path_sideVeerRight, PathForward, -90);
					autoState++;
					break;
				case 1:
					if(advancedAutoDrive()) {
						if(plateColour[0] == 'L')
							autoState++;
					}
					break;
				case 2:
					m_conveyor->SetSpeed(1.f);
					m_upperIntakeL->Set(ControlMode::PercentOutput, -1.f);
					m_upperIntakeR->Set(ControlMode::PercentOutput, 1.f);
					break;
				}
				break;
			case 5: //drive up right side, if corresponding side, run auto 3, if not, cross to left side
				switch(plateColour[0]) {
				case 'L':
					switch(autoState) {
					case 0:
						BBYCAKES->initPath(path_sideCrossLeft, PathForward, -270);
						autoState++;
						break;
					case 1:
						if(advancedAutoDrive())
							autoState++;
						break;
					case 2:
						m_conveyor->SetSpeed(1.f);
						m_upperIntakeL->Set(ControlMode::PercentOutput, -1.f);
						m_upperIntakeR->Set(ControlMode::PercentOutput, 1.f);
						break;
					}
					break;
				case 'R':
					autoMode = 3;
				}
				break;
			case 6: //drive up left side, if corresponding side, run auto 4, if not, cross to right side
				switch(plateColour[0]) {
				case 'R':
					switch(autoState) {
					case 0:
						BBYCAKES->initPath(path_sideCrossRight, PathForward, 270);
						autoState++;
						break;
					case 1:
						if(advancedAutoDrive())
							autoState++;
						break;
					case 2:
						m_conveyor->SetSpeed(1.f);
						m_upperIntakeL->Set(ControlMode::PercentOutput, -1.f);
						m_upperIntakeR->Set(ControlMode::PercentOutput, 1.f);
						break;
					}
					break;
				case 'L':
					autoMode = 4;
					break;
				}
				break;
			case 10: //cross auto line from right side or left side
				switch(autoState) {
				case 0:
					BBYCAKES->initPath(path_crossAutoLine, PathForward, 0);
					autoState++;
					break;
				case 1:
					advancedAutoDrive();
					break;
				}
				break;
			case 11: //exchange from centre
				switch(autoState) {
				case 0:
					BBYCAKES->initPath(path_exchange, PathForward, -180);
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
			case 12: //centre switch auto, then exchange
				switch(autoState) {
				case 0:
					autoRunTwelve = true;
					autoMode = 1;
					break;

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
		switch(driveState) {
		case 1:
			arcadeDrive();
			arcadeShift();
			break;
		case 2:
			cheezyDrive();
			cheezyShift();
			applesGripper();
			break;
		}
		operateConveyor();
		applesServo();
	}

	void arcadeShift() {
		if(m_Joystick->GetRawButton(1)) {
			m_shiftLow->Set(true);
			m_shiftHigh->Set(false);
		}
		else {
			m_shiftLow->Set(false);
			m_shiftHigh->Set(true);
		}
	}

	void cheezyShift() {
		if(m_GamepadOp->GetAButton() && !shiftToggleState1) {
			m_shiftLow->Set(true);
			m_shiftHigh->Set(false);
			shiftToggleState2 = true;
		}
		else if(!m_GamepadOp->GetAButton() && shiftToggleState2)
			shiftToggleState1 = true;
		else if(m_GamepadOp->GetAButton() && shiftToggleState1) {
			m_shiftLow->Set(false);
			m_shiftHigh->Set(true);
			shiftToggleState2 = false;
		}
		else if(!m_GamepadOp->GetAButton() && !shiftToggleState2)
			shiftToggleState1 = false;
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

	void lowerIntake() {
		switch(lowerIntakeState) {
		case 0:
			float intakeLSpeed = limit(m_GamepadDr->GetTriggerAxis(XboxController::kLeftHand));
			float intakeRSpeed = -limit(m_GamepadDr->GetTriggerAxis(XboxController::kRightHand));

			m_lowerIntakeL->SetSpeed(intakeLSpeed);
			m_lowerIntakeR->SetSpeed(intakeRSpeed);
			if(m_GamepadDr->GetXButton()) {
				lowerIntakeState = 10;
				intakeTimer->Start();
			}
			break;
		case 10:
			m_lowerIntakeL->SetSpeed(-1.f);
			m_lowerIntakeR->SetSpeed(1.f);
			if(intakeTimer->Get() > 2.f) {
				lowerIntakeState = 0;
				intakeTimer->Reset();
				intakeTimer->Stop();
			}
			break;
		}
	}

	void operateConveyor() {
		float conveyorFSpeed = limit(m_GamepadOp->GetTriggerAxis(XboxController::kLeftHand));
		float conveyorRSpeed = limit(m_GamepadOp->GetTriggerAxis(XboxController::kRightHand));

		m_conveyor->SetSpeed(conveyorFSpeed - conveyorRSpeed);
		m_upperIntakeL->Set(ControlMode::PercentOutput, conveyorFSpeed - conveyorRSpeed);
		m_upperIntakeR->Set(ControlMode::PercentOutput, -conveyorFSpeed + conveyorRSpeed);
	}

	void applesServo() {
		if(m_GamepadOp->GetBumper(XboxController::kRightHand))
			m_tailgateServo->SetAngle(90);
		else
			m_tailgateServo->SetAngle(0);
	}

	void applesGripper() {
		if(m_GamepadDr->GetBumper(XboxController::kRightHand)) {
			m_gripperExtend->Set(true);
			m_gripperRetract->Set(false);
		}
		else {
			m_gripperExtend->Set(false);
			m_gripperRetract->Set(true);
		}

		if(m_GamepadDr->GetBumper(XboxController::kLeftHand) && !intakeToggleState1) {
			m_gripperUp->Set(true);
			m_gripperDown->Set(false);
			intakeToggleState2 = true;
		}
		else if(!m_GamepadDr->GetBumper(XboxController::kLeftHand) && intakeToggleState2)
			intakeToggleState1 = true;
		else if(m_GamepadDr->GetBumper(XboxController::kLeftHand) && intakeToggleState1) {
			m_gripperUp->Set(false);
			m_gripperDown->Set(true);
			intakeToggleState2 = false;
		}
		else if(!m_GamepadDr->GetBumper(XboxController::kLeftHand) && !intakeToggleState2)
			intakeToggleState1 = false;
	}

	void cheezyDrive() {
		float joy1Y;
		float joy2X;

		switch(cheezyState) {
		case 1:
			joy1Y = -limit(expo(m_GamepadDr->GetRawAxis(1), 2)); // Getting the X position from the joystick
			joy2X = -limit(expo(m_GamepadDr->GetRawAxis(4), 3)); // Getting the Y position from the joystick
			break;
		case 2:
			joy1Y = -limit(expo(m_GamepadDr->GetRawAxis(5), 2)); // Getting the X position from the joystick
			joy2X = -limit(expo(m_GamepadDr->GetRawAxis(0), 3)); // Getting the Y position from the joystick
			break;
		}

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
