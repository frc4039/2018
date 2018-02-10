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
/*~~~~~Code function Index~~~~~~
 * Pathfollow Variables #106
 * RobotInit #119
 * Define Auto Paths #191
 * DisabledPeriodic #283
 * AutonomousInit #316
 * AutonomousPeriodic #334
 * TeleopInit #607
 * TeleopPeriodic #616
 * arcadeShift #637
 * cheezyShift #649
 * advancedAutoDrive #667
 * arcadeDrive #680
 * cheezyIntake #696
 * arcadeIntake #722
 * operateConveyor #731
 * applesServo #757
 * cheezyGripperPneumatics #764
 * arcadeGripperPneumatics #791
 * cheezyDrive #811
 * climber #833
 */

#define GP_L 5 //GamePad left bumper
#define GP_R 6 //GamePad Right bumper
#define GP_UP 0 //GamePad D-Pad(pLus thing) up
#define GP_DOWN 180 //GamePad D-Pad down. 0 is up and 180 is down.

#define CONVEYOR_SPEED -1.f
#define UPPER_SPEED 0.5f
#define LOWER_SPEED 0.5f

#define APPLES 14

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

	int autoState, autoMode, autoDelay, driveState, cheezyState, lowerIntakeState, conveyorState;
	bool shiftToggleState1, shiftToggleState2, intakeToggleState1, intakeToggleState2, climberToggleState1, climberToggleState2, autoRunTwelve;
	float currentGTime;

	XboxController *m_GamepadOp;
	XboxController *m_GamepadDr;

	Solenoid *m_gripperRetract;
	Solenoid *m_gripperExtend;
	Solenoid *m_gripperUp;
	Solenoid *m_gripperDown;
	Solenoid *m_squareExtend;
	Solenoid *m_squareRetract;

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
	Timer *gripperTimer;

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
	//exchange after center auto
	Path *path_backupEXRight, *path_backupEXLeft, *path_exchangeRight, *path_exchangeLeft;

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
		m_LFMotor = new VictorSP(9); // The place where you initialize your variables to their victors and which port on the RoboRio/computer
		m_LFMotor->SetSafetyEnabled(true);
		m_LBMotor = new VictorSP(8); // ^
		m_LBMotor->SetSafetyEnabled(true);
		m_RFMotor = new VictorSP(7); // ^
		m_RFMotor->SetSafetyEnabled(true);
		m_RBMotor = new VictorSP(6); // ^
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
		m_gripperUp = new Solenoid(5);
		m_gripperDown = new Solenoid(4);
		m_squareExtend = new Solenoid(6);
		m_squareRetract = new Solenoid(7);

		m_tailgateServo = new Servo(4);

		m_leftEncoder = new Encoder(0,1);
		m_rightEncoder = new Encoder(2,3);

		nav = new AHRS(SPI::Port::kMXP);
		nav->Reset();

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

		gripperTimer = new Timer();
		gripperTimer->Reset();
		gripperTimer->Stop();

		driveState = 1;
		cheezyState = 1;
		lowerIntakeState = 0;
		autoMode = 0;
		autoState = 0;
		autoDelay = 0;
		conveyorState = 0;
		shiftToggleState1 = false;
		shiftToggleState2 = false;
		climberToggleState1 = false;
		climberToggleState2 = false;
		autoRunTwelve = false;

		//================Define Auto Paths===============
		int zero[2] = {0, 0}; //starting point for all auto cases.

		int centreLeftEnd[2] = {6400, -7000};
		int cp1[2] = {0, -7000};
		int cp2[2] = {6400, 0};
		path_centreSwitchLeft = new PathCurve(zero, cp1, cp2, centreLeftEnd, 40);

		int centreRightEnd[2] = {6400, 6400};
		int cp3[2] = {0, 6400};
		int cp4[2] = {6400, 0};
		path_centreSwitchRight = new PathCurve(zero, cp3, cp4, centreRightEnd, 40);

		int centreLeftEnd2[2] = {9500, -5400};
		int cp5[2] = {1000, 0};
		int cp6[2] = {7500, -5400};
		path_centreSwitchLeft2 = new PathCurve(zero, cp5, cp6, centreLeftEnd2, 40);
		int cp7[2] = {4400, -7600};
		path_backupLeft = new PathCurve(centreLeftEnd2, cp6, cp7, centreLeftEnd, 40);

		int centreRightEnd2[2] = {9500, 5400};
		int cp8[2] = {1000, 0};
		int cp9[2] = {7500, 5400};
		path_centreSwitchRight2 = new PathCurve(zero, cp8, cp9, centreRightEnd2, 40);
		int cp10[2] = {4400, -7000};
		path_backupRight = new PathCurve(centreRightEnd2, cp9, cp10, centreRightEnd, 40);

		int sideVLEnd[2] = {6400, 0};
		int cp11[2] = {2000, 800};
		int cp12[2] = {14000, 0};
		path_sideVeerLeft = new PathCurve(zero, cp11, cp12, sideVLEnd, 60);

		int sideVREnd[2] = {13700, 2700};
		int cp13[2] = {4000, -2000};
		path_sideVeerRight = new PathCurve(zero, cp13, cp12, sideVREnd, 40);

		int sideCLEnd[2] = {6400, -15000};
		int cp14[2] = {7500, 800};
		int cp15[2] = {7400, -16000};
		path_sideCrossLeft = new PathCurve(zero, cp14, cp15, sideCLEnd, 40);

		int sideCREnd[2] = {6400, -15000};
		int cp16[2] = {7500, -800};
		int cp17[2] = {7400, 16000};
		path_sideCrossRight = new PathCurve(zero, cp16, cp17, sideCREnd, 40);

		int crossAutoEnd[2] = {13000, 0};
		path_crossAutoLine = new PathLine(zero, crossAutoEnd, 10);

		int exchangeEnd[2] = {0, -800};
		int cp18[2] = {800, 0};
		int cp19[2] = {800, -800};
		path_exchange = new PathCurve(zero, cp18, cp19, exchangeEnd, 40);

		int backupEXLeftEnd[2] = {7000, -8000};
		int cp20[2] = {6400, -7500};
		int cp21[2] = {6700, -7750};
		path_backupEXLeft = new PathCurve(centreLeftEnd, cp20, cp21, backupEXLeftEnd, 40);

		int backupEXRightEnd[2] = {7400, 7400};
		int cp22[2] = {7400,6900};
		int cp23[2] = {6900,6450};
		path_backupEXRight = new PathCurve(centreRightEnd, cp22, cp23, backupEXRightEnd, 40);

		int exchangeLeft[2] = {7000, -8000};
		int cp26[2] = {1,2};
		int cp27[2] = {3,4};
		path_exchangeLeft = new PathCurve(backupEXLeftEnd, cp26, cp27, exchangeLeft, 40);

		int exchangeRight[2] = {7400, 7400};
		int cp24[2] = {5,6};
		int cp25[2] = {7,8};
		path_exchangeRight = new PathCurve(backupEXRightEnd, cp24, cp25, exchangeRight, 40);

		m_turnPID = new SimPID(0.4, 0, 0.125, 0, 0.9);
		m_turnPID->setContinuousAngle(true);
		m_drivePID = new SimPID(0.0005, 0, 0, 0, 200);
		m_drivePID->setMaxOutput(0.8);
		m_finalTurnPID = new SimPID(0.2, 0, 0.05, 0, 0.9);
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
				BBYCAKES->reset();
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

		if(m_GamepadDr->GetBumper(XboxController::kLeftHand) && m_GamepadDr->GetBumper(XboxController::kRightHand))
			driveState = 2;

		autoDelay = -5*(m_Joystick->GetRawAxis(3) - 1);
		DriverStation::ReportError("Left Encoder: " + std::to_string(LRead) + " | Right Encoder: " + std::to_string(RRead) + " | Gyro: " + std::to_string(GRead));
		DriverStation::ReportError("Auto Mode: " + std::to_string(autoMode) + " | Auto Delay: " + std::to_string(autoDelay));
		DriverStation::ReportError("Drive State: " + std::to_string(driveState) + " | Cheezy State: " + std::to_string(cheezyState));

		BBYCAKES->updatePos(m_leftEncoder->Get(), m_rightEncoder->Get(), nav->GetYaw());
		printf("robot position x: %d\ty:%d\n", BBYCAKES->getXPos(), BBYCAKES->getYPos());
	}

	void AutonomousInit() override {
		m_LFMotor->SetSpeed(0.f);
		m_LBMotor->SetSpeed(0.f);
		m_RFMotor->SetSpeed(0.f);
		m_RBMotor->SetSpeed(0.f);
		m_upperIntakeL->Set(ControlMode::PercentOutput, 0.f);
		m_upperIntakeR->Set(ControlMode::PercentOutput, 0.f);
		m_conveyor->SetSpeed(0.f);
		m_lowerIntakeL->SetSpeed(0.f);
		m_lowerIntakeR->SetSpeed(0.f);
		m_gripperDown->Set(true);
		m_gripperUp->Set(false);
		m_squareExtend->Set(true);
		m_squareRetract->Set(false);
		m_gripperRetract->Set(false);
		m_gripperExtend->Set(true);
		m_leftEncoder->Reset();
		m_rightEncoder->Reset();
		nav->Reset();
		m_shiftLow->Set(true);
		m_shiftHigh->Set(false);
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
						if(autoTimer->Get() > 2.f)
							autoState++;
						m_conveyor->Set(1.f);
						m_upperIntakeL->Set(ControlMode::PercentOutput, -1.f);
						m_upperIntakeR->Set(ControlMode::PercentOutput, 1.f);
						BBYCAKES->initPath(path_backupEXLeft, PathBackward, 120);
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
						if(autoTimer->Get() > 2.f)
							autoState++;
						m_conveyor->Set(1.f);
						m_upperIntakeL->Set(ControlMode::PercentOutput, -1.f);
						m_upperIntakeR->Set(ControlMode::PercentOutput, 1.f);
						BBYCAKES->initPath(path_backupEXRight, PathBackward, -120);
						break;
					}
					break;
				}
				switch(autoState) {
				case 3:
					m_conveyor->Set(0.f);
					m_upperIntakeL->Set(ControlMode::PercentOutput, 0.f);
					m_upperIntakeR->Set(ControlMode::PercentOutput, 0.f);
					if(autoRunTwelve)
						autoMode = 12;
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
						m_upperIntakeL->Set(ControlMode::PercentOutput, -UPPER_SPEED);
						m_upperIntakeR->Set(ControlMode::PercentOutput, -UPPER_SPEED);
						m_conveyor->SetSpeed(CONVEYOR_SPEED);
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
					autoTimer->Reset();
					break;
				case 1:
					if(advancedAutoDrive() || autoTimer->Get() > 10.f) {
						autoTimer->Reset();
						if(plateColour[0] == 'R')
							autoState++;
					}
					break;
				case 2:
					if(autoTimer->Get() < 0.3) {
						m_LFMotor->SetSpeed(-0.9f);
						m_LBMotor->SetSpeed(-0.9f);
						m_RFMotor->SetSpeed(0.9f);
						m_RFMotor->SetSpeed(0.9f);
					}
					m_conveyor->SetSpeed(CONVEYOR_SPEED);
					m_upperIntakeL->Set(ControlMode::PercentOutput, -UPPER_SPEED);
					m_upperIntakeR->Set(ControlMode::PercentOutput, -UPPER_SPEED);
					break;
				}
				break;
			case 4: //drive forward from left side and veer right, deploy cube if corresponding side
				switch(autoState) {
				case 0:
					BBYCAKES->initPath(path_sideVeerRight, PathForward, 90);
					autoState++;
					autoTimer->Reset();
					break;
				case 1:
					if(advancedAutoDrive() || autoTimer->Get() > 10.f) {
						autoTimer->Reset();
						if(plateColour[0] == 'L')
							autoState++;
					}
					break;
				case 2:
					if(autoTimer->Get() < 0.3) {
						m_LFMotor->SetSpeed(-0.9f);
						m_LBMotor->SetSpeed(-0.9f);
						m_RFMotor->SetSpeed(0.9f);
						m_RFMotor->SetSpeed(0.9f);
					}
					m_conveyor->SetSpeed(CONVEYOR_SPEED);
					m_upperIntakeL->Set(ControlMode::PercentOutput, -1.f);
					m_upperIntakeR->Set(ControlMode::PercentOutput, -1.f);
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
				case 3:
					if(advancedAutoDrive())
						autoState++;
					break;
				}
				switch(plateColour[0]) {
				case 'L':
					switch(autoState) {
					case 4:
						BBYCAKES->initPath(path_exchangeLeft, PathForward, 180);
						autoState++;
						break;
					case 5:
						if(advancedAutoDrive())
							autoState++;
						m_lowerIntakeL->SetSpeed(1.f);
						m_lowerIntakeR->SetSpeed(-1.f);
						break;
					case 6:
						m_lowerIntakeL->SetSpeed(-1.f);
						m_lowerIntakeR->SetSpeed(1.f);
						break;
					}
					break;
				case 'R':
					switch(autoState) {
					case 4:
						BBYCAKES->initPath(path_exchangeRight, PathForward, -180);
						autoState++;
						break;
					case 5:
						if(advancedAutoDrive())
							autoState++;
						m_lowerIntakeL->SetSpeed(1.f);
						m_lowerIntakeR->SetSpeed(-1.f);
						break;
					case 6:
						m_lowerIntakeL->SetSpeed(-1.f);
						m_lowerIntakeR->SetSpeed(1.f);
						break;
					}
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
		m_upperIntakeL->Set(ControlMode::PercentOutput, 0.f);
		m_upperIntakeR->Set(ControlMode::PercentOutput, 0.f);
		m_conveyor->SetSpeed(0.f);
		m_lowerIntakeL->SetSpeed(0.f);
		m_lowerIntakeR->SetSpeed(0.f);
		m_gripperDown->Set(true);
		m_gripperUp->Set(false);
		gripperTimer->Reset();
		gripperTimer->Start();
		currentGTime = 0.f;
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
			cheezyIntake();
			cheezyGripperPneumatics();
			break;
		}
		operateIntake();
		operateGripperPneumatics();
		climber();
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
		if(m_GamepadDr->GetAButton() && !shiftToggleState1) {
			m_shiftLow->Set(true);
			m_shiftHigh->Set(false);
			shiftToggleState2 = true;
		}
		else if(!m_GamepadDr->GetAButton() && shiftToggleState2)
			shiftToggleState1 = true;
		else if(m_GamepadDr->GetAButton() && shiftToggleState1) {
			m_shiftLow->Set(false);
			m_shiftHigh->Set(true);
			shiftToggleState2 = false;
		}
		else if(!m_GamepadDr->GetAButton() && !shiftToggleState2)
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

	void cheezyIntake() {
		float intakeLSpeed = limit(m_GamepadDr->GetTriggerAxis(XboxController::kLeftHand));
		float intakeRSpeed = -limit(m_GamepadDr->GetTriggerAxis(XboxController::kRightHand));

		switch(lowerIntakeState) {
		case 0:
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

	void operateIntake() {
		float intakeFSpeed = limit(m_GamepadOp->GetTriggerAxis(XboxController::kLeftHand));
		float intakeRSpeed = limit(m_GamepadOp->GetTriggerAxis(XboxController::kRightHand));

		m_lowerIntakeL->SetSpeed(intakeFSpeed - intakeRSpeed);
		m_lowerIntakeR->SetSpeed(intakeFSpeed - intakeRSpeed);
	}

	void operateConveyor() {
		float conveyorJoy = limit(m_GamepadOp->GetY(XboxController::kLeftHand));

		switch(conveyorState) {
		case 0:
			if(m_GamepadOp->GetAButton())
				conveyorState = 10;

			m_upperIntakeL->Set(ControlMode::PercentOutput, 0.f);
			m_upperIntakeR->Set(ControlMode::PercentOutput, 0.f);

			m_conveyor->SetSpeed(conveyorJoy);
			break;
		case 10:
			if(!m_GamepadOp->GetAButton())
				conveyorState = 0;

			m_conveyor->SetSpeed(CONVEYOR_SPEED);
			m_upperIntakeL->Set(ControlMode::PercentOutput, -UPPER_SPEED);
			m_upperIntakeR->Set(ControlMode::PercentOutput, -UPPER_SPEED);
			break;
		}
	}

	void applesServo() {
		if(m_GamepadOp->GetBumper(XboxController::kRightHand))
			m_tailgateServo->SetAngle(90);
		else
			m_tailgateServo->SetAngle(0);
	}

	void cheezyGripperPneumatics() {
		if(m_GamepadDr->GetBumper(XboxController::kRightHand)) {
			m_gripperRetract->Set(true);
			m_gripperExtend->Set(false);
		}
		else {
			m_gripperRetract->Set(false);
			m_gripperExtend->Set(true);
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

	void operateGripperPneumatics() {
		if(m_GamepadOp->GetBButton()) {
			m_gripperRetract->Set(true);
			m_gripperExtend->Set(false);
			m_squareExtend->Set(false);
			m_squareRetract->Set(true);
			currentGTime = gripperTimer->Get();
		}
		else {
			m_squareExtend->Set(true);
			m_squareRetract->Set(false);
			if(gripperTimer->Get() > currentGTime + 0.5f) {
				m_gripperRetract->Set(false);
				m_gripperExtend->Set(true);
			}
		}

		if(m_GamepadOp->GetPOV(0) == GP_UP) {
			m_gripperUp->Set(true);
			m_gripperDown->Set(false);
		}
		else if(m_GamepadOp->GetPOV(0) == GP_DOWN) {
			m_gripperUp->Set(false);
			m_gripperDown->Set(true);
		}
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

	void climber() {

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
