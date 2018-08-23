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
#define UPPER_SPEED 0.4f
#define LOWER_SPEED 0.5f
#define TALON_TIMEOUT 10
#define TALON_LOOP_ID 0
#define DEFAULT_MAXOUT 0.68

#define APPLES 14
#define CURVE_RES 80
#define LINE_RES 10
#define SERVO_HOME -90.f

//#define AUX_PWM
//#define PRACTICE_BOT
//#define HAIL_MARY
#define STRETCH_ENABLED

#define STRETCH_UP 642000
#define STRETCH_TILT 149000
#define STRETCH_HALF

class Robot: public frc::IterativeRobot {
public:

	//declare class members/variables
	PowerDistributionPanel *m_PDP;

	Joystick *m_Joystick;
	Joystick *m_Joystick2;

	VictorSP *m_LFMotor, *m_LBMotor, *m_RFMotor, *m_RBMotor;
	VictorSP *m_lowerIntakeL, *m_lowerIntakeR;
	VictorSP *m_conveyor;

	TalonSRX *m_upperIntakeL, *m_upperIntakeR;
	TalonSRX *m_climber1;
	TalonSRX *m_stretchExtend;
	VictorSPX *m_stretchIntake;

	Solenoid *m_shiftLow;
	Solenoid *m_shiftHigh;

	int autoState, autoMode, autoDelay, conveyorState, indicatorState, lowerIntakeState, stretchState;
	bool twoCubeMode, currentError, joyBlues, autoRecord;
	float currentGTime, switchShotSpeed, servoHomeAngle;

	XboxController *m_GamepadOp;
	XboxController *m_GamepadDr;

	Solenoid *m_gripperExtend;
	Solenoid *m_gripperRetract;
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
	Timer *indicatorTimer;
	Timer *recordTimer;
	Timer *stretchTimer;
	int printDelay;

	Servo *m_tailgateServo;

	DigitalInput *m_beamSensorLower;

	cs::UsbCamera cam0;
//	cs::UsbCamera cam1;

	Relay *m_mangoRingLight;

	std::ofstream autoFile1;
	std::string line;
	std::ifstream playFile;
	float autoJoyY;
	float autoJoyX;
	bool autoJoy1;
	bool autoJoy9;
	bool autoJoy10;
	bool autoGameB;
	float autoGameRT;
	int autoGamePOV;
	int autoIntakeState;

	//====================Pathfollow Variables==================
	PathFollower *METRO;

	//drive to switch, drop cube on ends
	Path *path_centreSwitchLeft, *path_centreSwitchRight;
	//drive to swtich, drop cube via on close face
	Path *path_centreSwitchLeft2, *path_centreSwitchRight2, *path_backupLeft, *path_backupRight;
	//drive to switch and stop from any station
	Path *path_sideVeerLeft, *path_sideVeerRight, *path_sideCrossLeft, *path_sideCrossRight;
	//drive past auto line
	Path *path_crossAutoLine, *path_crossNullZone;
	//exchange
	Path *path_exchange;
	//exchange after center auto
	Path *path_backupEXRight, *path_backupEXLeft, *path_exchangeRight, *path_exchangeLeft;
	//two cube auto
	Path *path_twoCubeBackupRight, *path_twoCubeBackupLeft, *path_twoCubePickup, *path_twoCubeBackupLine, *path_twoCubeShootRight, *path_twoCubeShootLeft;
	//Scale auto
	Path *path_scaleRight, *path_scaleLeft;

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
		//m_PDP = new PowerDistributionPanel(0);

		m_LFMotor = new VictorSP(9); // The place where you initialize your variables to their victors and which port on the RoboRio/computer
		m_LBMotor = new VictorSP(8); // ^
		m_RFMotor = new VictorSP(7); // ^
		m_RBMotor = new VictorSP(6); // ^

		cam0 = CameraServer::GetInstance()->StartAutomaticCapture(0);
		cam0.SetExposureManual(50);
//		cam1 = CameraServer::GetInstance()->StartAutomaticCapture(1);
	//	cam1.SetExposureManual(50);

		m_lowerIntakeL = new VictorSP(1); // left intake
		m_lowerIntakeR = new VictorSP(2); // right intake

		m_conveyor = new VictorSP(0);
		m_upperIntakeL = new TalonSRX(1);
		m_upperIntakeR = new TalonSRX(2);

//		m_climber1 = new TalonSRX(3);
		m_stretchIntake = new VictorSPX(4);

		m_stretchExtend = new TalonSRX(5);
#ifdef STRETCH_ENABLED
//		int absolutePosition = m_stretchExtend->GetSelectedSensorPosition(TALON_LOOP_ID);
		m_stretchExtend->SetSelectedSensorPosition(FeedbackDevice::CTRE_MagEncoder_Relative, TALON_LOOP_ID, TALON_TIMEOUT);
		m_stretchExtend->ConfigNominalOutputForward(0.f, TALON_TIMEOUT);
		m_stretchExtend->ConfigNominalOutputReverse(0.f, TALON_TIMEOUT);
		m_stretchExtend->ConfigPeakOutputForward(1.f, TALON_TIMEOUT);
		m_stretchExtend->ConfigPeakOutputReverse(-0.1f, TALON_TIMEOUT);
		m_stretchExtend->Config_kF(TALON_LOOP_ID, 0, TALON_TIMEOUT);
		m_stretchExtend->Config_kP(TALON_LOOP_ID, 0.1, TALON_TIMEOUT);
		m_stretchExtend->Config_kI(TALON_LOOP_ID, 0, TALON_TIMEOUT);
		m_stretchExtend->Config_kD(TALON_LOOP_ID, 2.0, TALON_TIMEOUT);
		m_stretchExtend->ConfigMotionAcceleration(3029, TALON_TIMEOUT);
		m_stretchExtend->ConfigMotionCruiseVelocity(3029, TALON_TIMEOUT);
#endif

//		m_stretchL = new TalonSRX(6);
	//	m_stretchR = new TalonSRX(7);

		m_Joystick = new Joystick(0); // ^
		m_Joystick2 = new Joystick(1);// ^

		m_GamepadOp = new XboxController(2);
		m_GamepadDr = new XboxController(3);

		m_shiftLow = new Solenoid(0);
		m_shiftHigh = new Solenoid(1);
		m_gripperRetract = new Solenoid(2);
		m_gripperExtend = new Solenoid(3);
		m_gripperUp = new Solenoid(5);
		m_gripperDown = new Solenoid(4);
		m_squareExtend = new Solenoid(7);
		m_squareRetract = new Solenoid(6);

		m_tailgateServo = new Servo(4);
		m_beamSensorLower = new DigitalInput(4);

		m_leftEncoder = new Encoder(0,1);
		m_rightEncoder = new Encoder(2,3);

		nav = new AHRS(SPI::Port::kMXP);
		nav->Reset();

		m_mangoRingLight = new Relay(3);

		autoTimer = new Timer();
		autoTimer->Reset();
		autoTimer->Stop();

		delayTimer = new Timer();
		delayTimer->Reset();
		delayTimer->Stop();

/*		triggerTimer = new Timer();
		triggerTimer->Reset();
		triggerTimer->Stop();*/

		intakeTimer = new Timer();
		intakeTimer->Reset();
		intakeTimer->Stop();

		gripperTimer = new Timer();
		gripperTimer->Reset();
		gripperTimer->Stop();

		indicatorTimer = new Timer();
		indicatorTimer->Reset();
		indicatorTimer->Stop();

		stretchTimer = new Timer();
		stretchTimer->Reset();
		stretchTimer->Stop();

		recordTimer = new Timer();
		recordTimer->Reset();
		recordTimer->Stop();
		printDelay = 0;

//		driveState = 1;
	//	cheezyState = 1;
		lowerIntakeState = 0;
		indicatorState = 0;
		stretchState = 0;
		autoMode = 0;
		autoState = 0;
		autoDelay = 0;
		conveyorState = 0;
		switchShotSpeed = 0.5;
		servoHomeAngle = 0.f;
//		shiftToggleState1 = false;
//		shiftToggleState2 = false;
//		climberToggleState1 = false;
//		climberToggleState2 = false;
//		autoRunTwelve = false;
		currentError = false;
		joyBlues = false;

		autoFile1.open("/home/lvuser/autoFile.csv");
//		autoFile1 << "joyY,joyX,jb1,jb9,jb10,gpB,gpRT,gpPOV\n";
		autoRecord = false;

		//================Define Auto Paths===============
		int zero[2] = {0, 0};

		int centreLeftEnd[2] = {6400, -7000};
		int cp1[2] = {0, -7000};
		int cp2[2] = {6400, 0};
		path_centreSwitchLeft = new PathCurve(zero, cp1, cp2, centreLeftEnd, 20);

		int centreRightEnd[2] = {6400, 6400};
		int cp3[2] = {0, 6400};
		int cp4[2] = {6400, 0};
		path_centreSwitchRight = new PathCurve(zero, cp3, cp4, centreRightEnd, CURVE_RES);

		int centreLeftEnd2[2] = {9500, -6400}; //was 9500, -6400
		int cp5[2] = {3000, 0};
		int cp6[2] = {3500, -6400}; //was 3500, -6400
		path_centreSwitchLeft2 = new PathCurve(zero, cp5, cp6, centreLeftEnd2, 20); //was 20
		int cp7[2] = {4400, -12700};
		int cp7_0[2] = {3500, -6400};
		int backupLEnd[2] = {24200, -12600};
		path_backupLeft = new PathCurve(centreLeftEnd2, cp7_0, cp7, backupLEnd, 20);

		int centreRightEnd2[2] = {9500, 4400}; //was 9500, 4400
		int cp8[2] = {2000, 0};
		int cp9[2] = {3500, 4400}; //was 3500, 4400
		path_centreSwitchRight2 = new PathCurve(zero, cp8, cp9, centreRightEnd2, 20); //was 20
		int cp10[2] = {4400, 10000};
		int cp10_0[2] = {3500, 4400};
		int backupREnd[2] = {24200, 10000};
		path_backupRight = new PathCurve(centreRightEnd2, cp10_0, cp10, backupREnd, 20);

		int sideVLEnd[2] = {15200, -2200};
		int cp11[2] = {5500, 0};
		int cp12[2] = {15200, 3500};
		path_sideVeerLeft = new PathCurve(zero, cp11, cp12, sideVLEnd, 40);

		int sideVREnd[2] = {15200, 2200};
		int cp13[2] = {5500, 0};
		int cp12_2[2] = {15200, -3500};
		path_sideVeerRight = new PathCurve(zero, cp13, cp12_2, sideVREnd, 40);

		int sideCLEnd[2] = {9500, -14000};
		int cp14[2] = {4700, 0};
		int cp15[2] = {4700, -14000};
		path_sideCrossLeft = new PathCurve(zero, cp14, cp15, sideCLEnd, CURVE_RES);

		int sideCREnd[2] = {9500, 14000};
		int cp16[2] = {4700, 0};
		int cp17[2] = {4700, 14000};
		path_sideCrossRight = new PathCurve(zero, cp16, cp17, sideCREnd, CURVE_RES);

		int crossAutoEnd[2] = {13000, 0};
		path_crossAutoLine = new PathLine(zero, crossAutoEnd, 2);
		int crossNullEnd[2] = {13000, 0};
		path_crossNullZone = new PathLine(zero, crossNullEnd, 2);

		int exchangeEnd[2] = {0, -800};
		int cp18[2] = {800, 0};
		int cp19[2] = {800, -800};
		path_exchange = new PathCurve(zero, cp18, cp19, exchangeEnd, CURVE_RES);

		int backupEXLeftEnd[2] = {7000, -8000};
		int cp20[2] = {6400, -7500};
		int cp21[2] = {6700, -7750};
		path_backupEXLeft = new PathCurve(centreLeftEnd, cp20, cp21, backupEXLeftEnd, CURVE_RES);

		int backupEXRightEnd[2] = {7400, 7400};
		int cp22[2] = {7400, 6900};
		int cp23[2] = {6900, 6450};
		path_backupEXRight = new PathCurve(centreRightEnd, cp22, cp23, backupEXRightEnd, CURVE_RES);

		int exchangeLeft[2] = {7000, -8000};
		int cp26[2] = {1,2};
		int cp27[2] = {3,4};
		path_exchangeLeft = new PathCurve(backupEXLeftEnd, cp26, cp27, exchangeLeft, CURVE_RES);

		int exchangeRight[2] = {7400, 7400};
		int cp24[2] = {5,6};
		int cp25[2] = {7,8};
		path_exchangeRight = new PathCurve(backupEXRightEnd, cp24, cp25, exchangeRight, CURVE_RES);

		int scaleRightEnd[2] = {23000, -1400};
		int cp40[2] = {5000, 0};
		int cp39[2] = {15000, 0};
		path_scaleRight = new PathCurve(zero, cp40, cp39, scaleRightEnd, 20);

		int backupTwoCubeEnd[2] = {4500, -1000}; //was 4500, -2000
		int cp28[2] = {5500, 4000}; //was 4000, 5000
		int cp29[2] = {6000, -1500}; //was 6000, -1000
		int cp282[2] = {6000, 000};
		int cp292[2] = {6500, 3000};
		int centreRightScore[2] = {9500, 3500};
		path_twoCubeBackupRight = new PathCurve(centreRightEnd2, cp28, cp29, backupTwoCubeEnd, CURVE_RES/2);
		path_twoCubeShootRight = new PathCurve(backupTwoCubeEnd, cp282, cp292, centreRightScore, CURVE_RES/2);

		int cp30[2] = {6000, -6000}; //was 4000, -6000
		int cp31[2] = {6000, -550}; //was 8000, -550
		int cp31_2[2] = {6500, -5500}; //was 6500, -4500
		int centreLeftScore[2] = {9500, -6000}; //was 9500, -5000
		path_twoCubeBackupLeft = new PathCurve(centreLeftEnd2, cp30, cp31, backupTwoCubeEnd, 40);
		path_twoCubeShootLeft = new PathCurve(backupTwoCubeEnd, cp282, cp31_2, centreLeftScore, 40);

		int pickupTwoCubeEnd[2] = {9000, -1000}; //was 9000, -1000
		path_twoCubePickup = new PathLine(backupTwoCubeEnd, pickupTwoCubeEnd, 4);
		path_twoCubeBackupLine = new PathLine(pickupTwoCubeEnd, backupTwoCubeEnd, 2);

#ifdef PRACTICE_BOT
		m_turnPID = new SimPID(0.55, 0, 3.0, 0.0, 0.5);
#else
		m_turnPID = new SimPID(0.3, 0, 1.0, 0.0, 0);
#endif
		m_turnPID->setContinuousAngle(true);
		m_drivePID = new SimPID(0.0008, 0, 0, 0, 200);
#ifdef PRACTICE_BOT
		m_drivePID->setMaxOutput(0.7);
#else
		m_drivePID->setMaxOutput(DEFAULT_MAXOUT);
#endif
		m_finalTurnPID = new SimPID(0.52, 0, 7.0, 0, 0.5);
		m_finalTurnPID->setContinuousAngle(true);

		METRO = new PathFollower(500, PI/3, m_drivePID, m_turnPID, m_finalTurnPID);
		METRO->setIsDegrees(true);
		//METRO->enableStartRamp();
		METRO->setStartRamp(0.35, 1500);
	}

	void DisabledPeriodic() {
		int LRead = m_leftEncoder->Get();
		int RRead = m_rightEncoder->Get();
		float GRead = nav->GetYaw();

		for(int i = 1; i <= 12; i++) {
			if(m_Joystick->GetRawButton(i))	{
				autoMode = i;
				nav->Reset();
				m_leftEncoder->Reset();
				m_rightEncoder->Reset();
				METRO->reset();
#ifdef STRETCH_ENABLED
				m_stretchExtend->SetSelectedSensorPosition(0, TALON_LOOP_ID, TALON_TIMEOUT);
#endif
			}
		}

		if(m_Joystick->GetPOV(0) == 0) {
			printf("RECORDING MODE ENABLED\n");
			autoRecord = true;
		}
		else if(m_Joystick->GetPOV(0) == 180) {
			printf("RECORDING MODE DISABLED\n");
			autoRecord = false;
		}

		autoDelay = -5*(m_Joystick->GetRawAxis(3) - 1);

		METRO->updatePos(m_leftEncoder->Get(), m_rightEncoder->Get(), nav->GetYaw());

		plateColour = DriverStation::GetInstance().GetGameSpecificMessage();

		printDelay++;
		if(printDelay > 25){
			printDelay = 0;
			if(m_beamSensorLower->Get())
				printf("BEAM SENSOR TRUE\n");
			printf("robot position x: %d\ty:%d\n", METRO->getXPos(), METRO->getYPos());
			printf("Left Encoder: %d | Right Encoder: %d | Gyro: %f\n", LRead, RRead, GRead);
#ifdef STRETCH_ENABLED
			printf("StretchPos: %d\n", m_stretchExtend->GetSelectedSensorPosition(TALON_LOOP_ID));
#endif
			printf("Auto Mode: %d | Auto Delay: %d\n\n\n", autoMode, autoDelay);
		}
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
		m_stretchExtend->Set(ControlMode::PercentOutput, 0.f);
//		m_stretchL->Set(ControlMode::PercentOutput, 0.f);
	//	m_stretchR->Set(ControlMode::PercentOutput, 0.f);

		#ifndef HAIL_MARY
		m_gripperDown->Set(true);
#endif
		stretchTimer->Start();
		m_gripperUp->Set(false);
		m_squareExtend->Set(false);
		m_squareRetract->Set(true);
		m_gripperExtend->Set(false);
		m_gripperRetract->Set(true);
		m_leftEncoder->Reset();
		m_rightEncoder->Reset();
		nav->Reset();
		m_shiftLow->Set(true);
		m_shiftHigh->Set(false);
		autoState = 0;
//		autoRunTwelve = false;
		twoCubeMode = false;
		autoTimer->Reset();
		autoTimer->Start();
		delayTimer->Reset();
		delayTimer->Start();
		plateColour = DriverStation::GetInstance().GetGameSpecificMessage();
	}

	void AutonomousPeriodic() {
		printf("automode: %d\tautoState: %d\n", autoMode, autoState);
		plateColour = DriverStation::GetInstance().GetGameSpecificMessage();

		if(delayTimer->Get() > autoDelay) {
//TODO
			switch(autoMode) {
			case 1: //recorded auto mode
				switch(autoState) {
				case 0:
					autoIntakeState = 0;
					autoJoyY = 0.f;
					autoJoyX = 0.f;
					autoJoy1 = false;
					autoJoy9 = false;
					autoJoy10 = false;
					autoGameB = false;
					autoGameRT = 0.f;
					autoGamePOV = NULL;
					playFile.open("/home/lvuser/autoFile.csv");
//					getline(playFile, line);
					autoState++;
					break;
				case 1:
					if (!playFile.eof()){
						getline(playFile, line, ',');
						autoJoyY = limit(std::stof(line));

						getline(playFile, line, ',');
						autoJoyX = limit(std::stof(line));

						getline(playFile, line, ',');
						autoJoy1 = std::stoi(line);

						getline(playFile, line, ',');
						autoJoy9 = std::stoi(line);

						getline(playFile, line, ',');
						autoJoy10 = std::stoi(line);

						getline(playFile, line, ',');
						autoGameB = std::stoi(line);

						getline(playFile, line, ',');
						autoGameRT = limit(std::stof(line));

						getline(playFile, line, '\n');
						autoGamePOV = std::stoi(line);
					}
					else
						autoState++;

					m_LFMotor->SetSpeed(autoJoyY - autoJoyX);
					m_LBMotor->SetSpeed(autoJoyY - autoJoyX);
					m_RFMotor->SetSpeed(-autoJoyY - autoJoyX);
					m_RBMotor->SetSpeed(-autoJoyY - autoJoyX);

					if(autoJoy1) {
						m_shiftLow->Set(true);
						m_shiftHigh->Set(false);
					}
					else {
						m_shiftLow->Set(false);
						m_shiftHigh->Set(true);
					}

					if(autoJoy10) {
						m_conveyor->SetSpeed(CONVEYOR_SPEED);
						m_upperIntakeL->Set(ControlMode::PercentOutput, UPPER_SPEED);
						m_upperIntakeR->Set(ControlMode::PercentOutput, UPPER_SPEED);
					}
					else {
						m_conveyor->SetSpeed(0.f);
						m_upperIntakeL->Set(ControlMode::PercentOutput, 0.f);
						m_upperIntakeR->Set(ControlMode::PercentOutput, 0.f);
					}

					if(autoGameB) {
						m_gripperExtend->Set(true);
						m_gripperRetract->Set(false);
						if(gripperTimer->Get() > 0.5f) {
							m_squareExtend->Set(true);
							m_squareRetract->Set(false);
						}
					}
					else {
						m_gripperExtend->Set(false);
						m_gripperRetract->Set(true);
						m_squareExtend->Set(false);
						m_squareRetract->Set(true);
						gripperTimer->Reset();
					}

					if(autoGamePOV == GP_UP) {
						m_gripperUp->Set(true);
						m_gripperDown->Set(false);
					}
					else if(autoGamePOV == GP_DOWN) {
						m_gripperUp->Set(false);
						m_gripperDown->Set(true);
					}

					switch(autoIntakeState) {
					case 0:
						if(m_squareExtend->Get()) {
							m_lowerIntakeL->SetSpeed(-limit2(-autoJoyY, 0.8, 0.3));
							m_lowerIntakeR->SetSpeed(-limit2(-autoJoyY, 0.8, 0.3));
						}
						else {
							m_lowerIntakeL->SetSpeed(-autoGameRT);
							m_lowerIntakeR->SetSpeed(-autoGameRT);
						}

						if(autoJoy9)
							autoIntakeState = 1;
						break;
					case 1:
						if(m_gripperDown->Get()) {
							m_lowerIntakeL->SetSpeed(1.f);
							m_lowerIntakeR->SetSpeed(1.f);
						}
						else {
							m_lowerIntakeL->SetSpeed(0.5f);
							m_lowerIntakeR->SetSpeed(0.5f);
						}

						if(!autoJoy9)
							autoIntakeState = 0;
						break;
					}
					break;
				case 2:
					m_LFMotor->SetSpeed(0.f);
					m_LBMotor->SetSpeed(0.f);
					m_RFMotor->SetSpeed(0.f);
					m_RBMotor->SetSpeed(0.f);
					m_upperIntakeL->Set(ControlMode::PercentOutput, 0.f);
					m_upperIntakeR->Set(ControlMode::PercentOutput, 0.f);
					m_conveyor->SetSpeed(0.f);
					m_lowerIntakeL->SetSpeed(0.f);
					m_lowerIntakeR->SetSpeed(0.f);
					playFile.close();
					break;
				}
				break;
			case 11:
				switch(autoState) {
				case 0:
					if(plateColour[1] == 'L')
						METRO->initPath(path_crossAutoLine, PathForward, 0);
					else
						METRO->initPath(path_crossNullZone, PathForward, 0);

					autoState++;
					break;
				case 1:
					advancedAutoDrive();
					break;
				}
				break;
			case 12:
				switch(autoState) {
				case 0:
					if(plateColour[1] == 'R')
						METRO->initPath(path_crossAutoLine, PathForward, 0);
					else
						METRO->initPath(path_crossNullZone, PathForward, 0);

					autoState++;
					break;
				case 1:
					advancedAutoDrive();
					break;
				}
				break;
			case 2: //deploy cube on close face of the switch
				switch(plateColour[0]) {
				case 'L':
					switch(autoState) {
					case 0:
//						m_drivePID->setMaxOutput(0.85);
						METRO->initPath(path_centreSwitchLeft2, PathForward, 0);
						autoState++;
						break;
					case 1:
						if(advancedAutoDrive()) {
							autoState++;
							autoTimer->Reset();
						}
						break;
					case 2:
						if(autoTimer->Get() < 0.5f) {
							m_LFMotor->SetSpeed(-0.5f);
							m_LBMotor->SetSpeed(-0.5f);
							m_RFMotor->SetSpeed(0.5f);
							m_RBMotor->SetSpeed(0.5f);
						}
						else {
							m_LFMotor->SetSpeed(0.f);
							m_LBMotor->SetSpeed(0.f);
							m_RFMotor->SetSpeed(0.f);
							m_RBMotor->SetSpeed(0.f);
						}
						m_upperIntakeL->Set(ControlMode::PercentOutput, UPPER_SPEED);
						m_upperIntakeR->Set(ControlMode::PercentOutput, UPPER_SPEED);
						m_conveyor->SetSpeed(CONVEYOR_SPEED);
#ifdef STRETCH_ENABLED
						m_stretchIntake->Set(ControlMode::PercentOutput, -UPPER_SPEED);
#endif
						if(!twoCubeMode)
							METRO->initPath(path_backupLeft, PathBackward, 180);

						if(autoTimer->Get() > 1.4f) {
							if(twoCubeMode) {
								METRO->initPath(path_twoCubeBackupLeft, PathBackward, 0);
								autoMode = 7;
								autoState++;
								gripperTimer->Reset();
								gripperTimer->Start();
							}

							if(plateColour[1] == 'L' && !twoCubeMode)
								autoState++;
							else {
								m_upperIntakeL->Set(ControlMode::PercentOutput, 0.f);
								m_upperIntakeR->Set(ControlMode::PercentOutput, 0.f);
								m_conveyor->SetSpeed(0.f);
#ifdef STRETCH_ENABLED
								m_stretchIntake->Set(ControlMode::PercentOutput, 0.f);
#endif
							}
						}
						break;
					case 3:
						m_upperIntakeL->Set(ControlMode::PercentOutput, 0.f);
						m_upperIntakeR->Set(ControlMode::PercentOutput, 0.f);
						m_conveyor->SetSpeed(0.f);
#ifdef STRETCH_ENABLED
						m_stretchIntake->Set(ControlMode::PercentOutput, 0.f);
#endif
						if(advancedAutoDrive())
							autoState++;
						break;
					case 4:
						m_LFMotor->SetSpeed(0.f);
						m_LBMotor->SetSpeed(0.f);
						m_RFMotor->SetSpeed(0.f);
						m_RBMotor->SetSpeed(0.f);
						break;
					}
					break;
				case 'R':
					switch(autoState) {
					case 0:
			//			m_drivePID->setMaxOutput(0.8);
						METRO->initPath(path_centreSwitchRight2, PathForward, 0);
						autoTimer->Reset();
						autoState++;
						break;
					case 1:
						if(advancedAutoDrive()) {
							autoState++;
							autoTimer->Reset();
						}
						break;
					case 2:
						if(autoTimer->Get() < 0.5f) {
							m_LFMotor->SetSpeed(-0.5f);
							m_LBMotor->SetSpeed(-0.5f);
							m_RFMotor->SetSpeed(0.5f);
							m_RBMotor->SetSpeed(0.5f);
						}
						else {
							m_LFMotor->SetSpeed(0.f);
							m_LBMotor->SetSpeed(0.f);
							m_RFMotor->SetSpeed(0.f);
							m_RBMotor->SetSpeed(0.f);
						}
						m_upperIntakeL->Set(ControlMode::PercentOutput, UPPER_SPEED);
						m_upperIntakeR->Set(ControlMode::PercentOutput, UPPER_SPEED);
						m_conveyor->SetSpeed(CONVEYOR_SPEED);
#ifdef STRETCH_ENABLED
						m_stretchIntake->Set(ControlMode::PercentOutput, -UPPER_SPEED);
#endif

						if(!twoCubeMode)
							METRO->initPath(path_backupRight, PathBackward, -180);

						if(autoTimer->Get() > 1.4f) {
							if(twoCubeMode) {
								METRO->initPath(path_twoCubeBackupRight, PathBackward, 0);
								autoMode = 7;
								autoState++;
								gripperTimer->Reset();
								gripperTimer->Start();
							}

							if(plateColour[1] == 'R' && !twoCubeMode)
								autoState++;
							else {
								m_upperIntakeL->Set(ControlMode::PercentOutput, 0.f);
								m_upperIntakeR->Set(ControlMode::PercentOutput, 0.f);
								m_conveyor->SetSpeed(0.f);
#ifdef STRETCH_ENABLED
								m_stretchIntake->Set(ControlMode::PercentOutput, 0.f);
#endif
							}
						}
						break;
					case 3:
						m_upperIntakeL->Set(ControlMode::PercentOutput, 0.f);
						m_upperIntakeR->Set(ControlMode::PercentOutput, 0.f);
						m_conveyor->SetSpeed(0.f);
#ifdef STRETCH_ENABLED
						m_stretchIntake->Set(ControlMode::PercentOutput, 0.f);
#endif
						if(advancedAutoDrive())
							autoState++;
						break;
					case 4:
						m_LFMotor->SetSpeed(0.f);
						m_LBMotor->SetSpeed(0.f);
						m_RFMotor->SetSpeed(0.f);
						m_RBMotor->SetSpeed(0.f);
						break;
					}
					break;
				}
				break;
			case 3: //drive forward from right side and veer left, deploy cube if corresponding side ...right side
				switch(plateColour[0]) {
				case 'R':
					switch(autoState) {
					case 0:
						METRO->initPath(path_sideVeerLeft, PathForward, -90);
						autoState++;
						autoTimer->Reset();
						break;
					case 1:
						if(advancedAutoDrive()) {
							autoTimer->Reset();
							autoState++;
						}
						break;
					case 2:
						if(autoTimer->Get() < 1.2f) {
							m_LFMotor->SetSpeed(-0.5f);
							m_LBMotor->SetSpeed(-0.5f);
							m_RFMotor->SetSpeed(0.5f);
							m_RFMotor->SetSpeed(0.5f);
						}
						else {
							m_LFMotor->SetSpeed(0.f);
							m_LBMotor->SetSpeed(0.f);
							m_RFMotor->SetSpeed(0.f);
							m_RBMotor->SetSpeed(0.f);
						}

						if(autoTimer->Get() < 3.f) {
							m_conveyor->SetSpeed(CONVEYOR_SPEED);
							m_upperIntakeL->Set(ControlMode::PercentOutput, 0.4f);
							m_upperIntakeR->Set(ControlMode::PercentOutput, 0.4f);
#ifdef STRETCH_ENABLED
							m_stretchIntake->Set(ControlMode::PercentOutput, -UPPER_SPEED);
#endif
						}
						else {
							m_conveyor->SetSpeed(0.f);
							m_upperIntakeL->Set(ControlMode::PercentOutput, 0.f);
							m_upperIntakeR->Set(ControlMode::PercentOutput, 0.f);
#ifdef STRETCH_ENABLED
							m_stretchIntake->Set(ControlMode::PercentOutput, 0.f);
#endif
						}
						break;
					}
					break;
				case 'L':
					autoMode = 11;
					break;
				}
				break;
			case 4: //drive forward from left side and veer right, deploy cube if corresponding side
				switch(plateColour[0]) {
				case 'L':
					switch(autoState) {
					case 0:
						METRO->initPath(path_sideVeerRight, PathForward, 90);
						autoState++;
						autoTimer->Reset();
						break;
					case 1:
						if(advancedAutoDrive() || autoTimer->Get() > 30.f) {
							autoTimer->Reset();
							autoState++;
						}
						break;
					case 2:
						if(autoTimer->Get() < 1.2f) {
							m_LFMotor->SetSpeed(-0.5f);
							m_LBMotor->SetSpeed(-0.5f);
							m_RFMotor->SetSpeed(0.5f);
							m_RFMotor->SetSpeed(0.5f);
						}
						else {
							m_LFMotor->SetSpeed(0.f);
							m_LBMotor->SetSpeed(0.f);
							m_RFMotor->SetSpeed(0.f);
							m_RBMotor->SetSpeed(0.f);
						}

						if(autoTimer->Get() < 3.f) {
							m_conveyor->SetSpeed(CONVEYOR_SPEED);
							m_upperIntakeL->Set(ControlMode::PercentOutput, 0.4f);
							m_upperIntakeR->Set(ControlMode::PercentOutput, 0.4f);
#ifdef STRETCH_ENABLED
							m_stretchIntake->Set(ControlMode::PercentOutput, -UPPER_SPEED);
#endif
						}
						else {
							m_conveyor->SetSpeed(0.f);
							m_upperIntakeL->Set(ControlMode::PercentOutput, 0.f);
							m_upperIntakeR->Set(ControlMode::PercentOutput, 0.f);
#ifdef STRETCH_ENABLED
							m_stretchIntake->Set(ControlMode::PercentOutput, 0.f);
#endif
						}
						break;
					}
					break;
				case 'R':
					autoMode = 12;
					break;
				}
				break;
			case 5: //drive up right side, if corresponding side, run auto 3, if not, cross to left side
				switch(plateColour[0]) {
				case 'L':
					switch(autoState) {
					case 0:
						METRO->initPath(path_sideCrossLeft, PathForward, 0);
						autoState++;
						break;
					case 1:
						if(advancedAutoDrive()) {
							autoState++;
							autoTimer->Reset();
							autoTimer->Start();
						}
						break;
					case 2:
						if(autoTimer->Get() < 1.2f) {
							m_LFMotor->SetSpeed(-0.5f);
							m_LBMotor->SetSpeed(-0.5f);
							m_RFMotor->SetSpeed(0.5f);
							m_RFMotor->SetSpeed(0.5f);
						}
						else {
							m_LFMotor->SetSpeed(0.f);
							m_LBMotor->SetSpeed(0.f);
							m_RFMotor->SetSpeed(0.f);
							m_RBMotor->SetSpeed(0.f);
						}
						m_conveyor->SetSpeed(1.f);
						m_upperIntakeL->Set(ControlMode::PercentOutput, 0.4f);
						m_upperIntakeR->Set(ControlMode::PercentOutput, 0.4f);
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
						METRO->initPath(path_sideCrossRight, PathForward, 0);
						autoState++;
						break;
					case 1:
						if(advancedAutoDrive()) {
							autoState++;
							autoTimer->Reset();
							autoTimer->Start();
						}
						break;
					case 2:
						if(autoTimer->Get() < 1.2f) {
							m_LFMotor->SetSpeed(-0.5f);
							m_LBMotor->SetSpeed(-0.5f);
							m_RFMotor->SetSpeed(0.5f);
							m_RFMotor->SetSpeed(0.5f);
						}
						else {
							m_LFMotor->SetSpeed(0.f);
							m_LBMotor->SetSpeed(0.f);
							m_RFMotor->SetSpeed(0.f);
							m_RBMotor->SetSpeed(0.f);
						}
						m_conveyor->SetSpeed(1.f);
						m_upperIntakeL->Set(ControlMode::PercentOutput, 0.4f);
						m_upperIntakeR->Set(ControlMode::PercentOutput, 0.4f);
						break;
					}
					break;
				case 'L':
					autoMode = 4;
					break;
				}
				break;
			case 7: //similar to auto 2, but grabs another cube. And shoots it. It's pretty cool if I do say so myself. apple
				switch(autoState) {
				case 0:
					twoCubeMode = true;
					autoMode = 2;
					break;
				case 3:
					m_gripperExtend->Set(true);
					m_gripperRetract->Set(false);
					m_upperIntakeL->Set(ControlMode::PercentOutput, 0.f);
					m_upperIntakeR->Set(ControlMode::PercentOutput, 0.f);
					m_conveyor->SetSpeed(0.f);
					if(advancedAutoDrive()) {
						autoState++;
						METRO->initPath(path_twoCubePickup, PathForward, 0);
//						m_drivePID->setMaxOutput(0.8f);
						autoTimer->Reset();
						autoTimer->Start();
					}
					break;
				case 4:

					m_lowerIntakeL->SetSpeed(-0.8f);
					m_lowerIntakeR->SetSpeed(-0.8f);
					m_squareExtend->Set(true);
					m_squareRetract->Set(false);

					advancedAutoDrive();
					if(!m_beamSensorLower->Get() || autoTimer->Get() > 2.f) {
						autoState++;
						METRO->initPath(path_twoCubeBackupLine, PathBackward, 0);
						//m_drivePID->setMaxOutput(0.9);
						autoTimer->Reset();
						autoTimer->Start();
					}
					break;
				case 5:
					m_gripperRetract->Set(true);
					m_gripperExtend->Set(false);
					m_squareExtend->Set(false);
					m_squareRetract->Set(true);
#ifdef STRETCH_ENABLED
					m_stretchExtend->Set(ControlMode::Position, STRETCH_TILT);
#endif
					if(autoTimer->Get() < 1.f && m_beamSensorLower->Get()) {
						m_lowerIntakeL->SetSpeed(-1.f);
						m_lowerIntakeR->SetSpeed(-1.f);
					}
					else {
						m_lowerIntakeL->SetSpeed(0.f);
						m_lowerIntakeR->SetSpeed(0.f);
					}

					if(advancedAutoDrive()) {
						switch(plateColour[0]) {
						case 'L':
							METRO->initPath(path_twoCubeShootLeft, PathForward, 0);
							break;
						case 'R':
							METRO->initPath(path_twoCubeShootRight, PathForward, 0);
							break;
						}
						autoState++;
						autoTimer->Reset();
					}
					break;
				case 6:
					m_lowerIntakeL->SetSpeed(-0.f);
					m_lowerIntakeR->SetSpeed(-0.f);
					m_gripperUp->Set(true);
					m_gripperDown->Set(false);
					if(advancedAutoDrive()) {
						m_LFMotor->SetSpeed(-0.5f);
						m_LBMotor->SetSpeed(-0.5f);
						m_RFMotor->SetSpeed(0.5f);
						m_RBMotor->SetSpeed(0.5f);
						autoTimer->Reset();
						autoState++;
					}
					break;
				case 7:
					if(autoTimer->Get() > 0.7f) {
						m_lowerIntakeL->SetSpeed(0.5f);
						m_lowerIntakeR->SetSpeed(0.5f);
						m_LFMotor->SetSpeed(0.f);
						m_LBMotor->SetSpeed(0.f);
						m_RFMotor->SetSpeed(0.f);
						m_RBMotor->SetSpeed(0.f);
					}
					if(autoTimer->Get() > 2.f) {
						m_lowerIntakeL->SetSpeed(0.f);
						m_lowerIntakeR->SetSpeed(0.f);
					}
				}
				break;
//TODO encoder values
			case 8: //scale auto from right side
				switch(plateColour[1]) {
				case 'R': //right side
					switch(autoState) {
					case 0:
						METRO->initPath(path_exchange, PathForward, -20); //who knows
						autoState++; //next case?
						break;
					case 1:
						m_stretchExtend->Set(ControlMode::Position, STRETCH_UP); //stretch extends
						if(advancedAutoDrive()) {
							autoState++; //next case
							autoTimer->Reset(); //reset timer
							autoTimer->Start(); //start timer
						}
						break;
					case 2:
						m_stretchIntake->Set(ControlMode::PercentOutput, 1.f); //run stretch intake
						m_gripperExtend->Set(true); //gripper is extended
						m_gripperRetract->Set(false);
						if(autoTimer->Get() > 1.5f) { //----------------only if doing switch stuff-------------------
							autoState++; //next case
							autoTimer->Reset(); //reset timer
					//		METRO->initPath(h, PathBackward, 90);
						}
						break;
//--------------------------------------------Switch part------------------------------------------------------------
					case 3:
						m_stretchExtend->Set(ControlMode::Position, STRETCH_TILT); //drop stretch halfway
						m_squareExtend->Set(true); //square is out
						m_squareRetract->Set(false);
						m_lowerIntakeL->SetSpeed(-0.8f); //intake at 80%
						m_lowerIntakeR->SetSpeed(-0.8f);
						advancedAutoDrive();
						if(!m_beamSensorLower->Get()) { //if on then switch state
							autoState++;
							autoTimer->Reset();
			//				METRO->initPath(h, PathForward, 180);
						}
						break;
					case 4:
						m_squareExtend->Set(false); //square is in
						m_squareRetract->Set(true);
						m_gripperExtend->Set(false); //gripper is in
						m_gripperRetract->Set(true);
						m_lowerIntakeL->SetSpeed(0.f); //intake off
						m_lowerIntakeR->SetSpeed(0.f);
						if(autoTimer->Get() < 0.3) { //
							m_LFMotor->SetSpeed(0.5f);
							m_LBMotor->SetSpeed(0.5f);
							m_RFMotor->SetSpeed(-0.5f);
							m_RBMotor->SetSpeed(-0.5f);
						}
						else if(autoTimer->Get() > 0.3 && autoTimer->Get() < 0.6) {
							m_LFMotor->SetSpeed(0.f);
							m_LBMotor->SetSpeed(0.f);
							m_RFMotor->SetSpeed(0.f);
							m_RBMotor->SetSpeed(0.f);
							m_gripperUp->Set(true);
							m_gripperDown->Set(false);
						}
						else
							autoState++;
						break;
					case 5:
						if(autoTimer->Get() < 0.4) {
							m_LFMotor->SetSpeed(0.5f);
							m_LBMotor->SetSpeed(0.5f);
							m_RFMotor->SetSpeed(-0.5f);
							m_RBMotor->SetSpeed(-0.5f);
						}
						else {
							m_LFMotor->SetSpeed(0.f);
							m_LBMotor->SetSpeed(0.f);
							m_RFMotor->SetSpeed(0.f);
							m_RBMotor->SetSpeed(0.f);
							m_lowerIntakeL->SetSpeed(0.5f);
							m_lowerIntakeR->SetSpeed(0.5f);
						}

						break;
					}
					break;
				case 'L':
					switch(autoState) {
					case 0:
						METRO->initPath(path_exchange, PathForward, -20); //who knows
						autoState++; //next case?
						break;
					case 1:
						m_stretchExtend->Set(ControlMode::Position, STRETCH_UP); //stretch extends
						if(advancedAutoDrive()) {
							autoState++; //next case
							autoTimer->Reset(); //reset timer
							autoTimer->Start(); //start timer
						}
						break;
					case 2:
						m_stretchIntake->Set(ControlMode::PercentOutput, 1.f); //run stretch intake
						m_gripperExtend->Set(true); //gripper is extended
						m_gripperRetract->Set(false);
						if(autoTimer->Get() > 1.5f) { //----------------only if doing switch stuff-------------------
							autoState++; //next case
							autoTimer->Reset(); //reset timer
					//		METRO->initPath(h, PathBackward, 90);
						}
						break;
					//--------------------------------------------Switch part------------------------------------------------------------
					case 3:
						m_stretchExtend->Set(ControlMode::Position, STRETCH_TILT); //drop stretch halfway
						m_squareExtend->Set(true); //square is out
						m_squareRetract->Set(false);
						m_lowerIntakeL->SetSpeed(-0.8f); //intake at 80%
						m_lowerIntakeR->SetSpeed(-0.8f);
						advancedAutoDrive();
						if(!m_beamSensorLower->Get()) { //if on then switch state
							autoState++;
							autoTimer->Reset();
			//				METRO->initPath(h, PathForward, 180);
						}
						break;
					case 4:
						m_squareExtend->Set(false); //square is in
						m_squareRetract->Set(true);
						m_gripperExtend->Set(false); //gripper is in
						m_gripperRetract->Set(true);
						m_lowerIntakeL->SetSpeed(0.f); //intake off
						m_lowerIntakeR->SetSpeed(0.f);
						if(autoTimer->Get() < 0.3) { //
							m_LFMotor->SetSpeed(0.5f);
							m_LBMotor->SetSpeed(0.5f);
							m_RFMotor->SetSpeed(-0.5f);
							m_RBMotor->SetSpeed(-0.5f);
						}
						else if(autoTimer->Get() > 0.3 && autoTimer->Get() < 0.6) {
							m_LFMotor->SetSpeed(0.f);
							m_LBMotor->SetSpeed(0.f);
							m_RFMotor->SetSpeed(0.f);
							m_RBMotor->SetSpeed(0.f);
							m_gripperUp->Set(true);
							m_gripperDown->Set(false);
						}
						else
							autoState++;
						break;
					case 5:
						if(autoTimer->Get() < 0.4) {
							m_LFMotor->SetSpeed(0.5f);
							m_LBMotor->SetSpeed(0.5f);
							m_RFMotor->SetSpeed(-0.5f);
							m_RBMotor->SetSpeed(-0.5f);
						}
						else {
							m_LFMotor->SetSpeed(0.f);
							m_LBMotor->SetSpeed(0.f);
							m_RFMotor->SetSpeed(0.f);
							m_RBMotor->SetSpeed(0.f);
							m_lowerIntakeL->SetSpeed(0.5f);
							m_lowerIntakeR->SetSpeed(0.5f);
						}

						break;
					}
					break;
				}
				break;
//TODO encoder values and starting position
			case 9: //scale auto from left side
				switch(plateColour[1]) {
				case 'R': //right side
					switch(autoState) {
					case 0:
						METRO->initPath(path_exchange, PathForward, -20); //who knows
						autoState++; //next case?
						break;
					case 1:
						m_stretchExtend->Set(ControlMode::Position, STRETCH_UP); //stretch extends
						if(advancedAutoDrive()) {
							autoState++; //next case
							autoTimer->Reset(); //reset timer
							autoTimer->Start(); //start timer
						}
						break;
					case 2:
						m_stretchIntake->Set(ControlMode::PercentOutput, 1.f); //run stretch intake
						m_gripperExtend->Set(true); //gripper is extended
						m_gripperRetract->Set(false);
						if(autoTimer->Get() > 1.5f) { //----------------only if doing switch stuff-------------------
							autoState++; //next case
							autoTimer->Reset(); //reset timer
					//		METRO->initPath(h, PathBackward, 90);
						}
						break;
//--------------------------------------------Switch part------------------------------------------------------------
					case 3:
						m_stretchExtend->Set(ControlMode::Position, STRETCH_TILT); //drop stretch halfway
						m_squareExtend->Set(true); //square is out
						m_squareRetract->Set(false);
						m_lowerIntakeL->SetSpeed(-0.8f); //intake at 80%
						m_lowerIntakeR->SetSpeed(-0.8f);
						advancedAutoDrive();
						if(!m_beamSensorLower->Get()) { //if on then switch state
							autoState++;
							autoTimer->Reset();
			//				METRO->initPath(h, PathForward, 180);
						}
						break;
					case 4:
						m_squareExtend->Set(false); //square is in
						m_squareRetract->Set(true);
						m_gripperExtend->Set(false); //gripper is in
						m_gripperRetract->Set(true);
						m_lowerIntakeL->SetSpeed(0.f); //intake off
						m_lowerIntakeR->SetSpeed(0.f);
						if(autoTimer->Get() < 0.3) { //
							m_LFMotor->SetSpeed(0.5f);
							m_LBMotor->SetSpeed(0.5f);
							m_RFMotor->SetSpeed(-0.5f);
							m_RBMotor->SetSpeed(-0.5f);
						}
						else if(autoTimer->Get() > 0.3 && autoTimer->Get() < 0.6) {
							m_LFMotor->SetSpeed(0.f);
							m_LBMotor->SetSpeed(0.f);
							m_RFMotor->SetSpeed(0.f);
							m_RBMotor->SetSpeed(0.f);
							m_gripperUp->Set(true);
							m_gripperDown->Set(false);
						}
						else
							autoState++;
						break;
					case 5:
						if(autoTimer->Get() < 0.4) {
							m_LFMotor->SetSpeed(0.5f);
							m_LBMotor->SetSpeed(0.5f);
							m_RFMotor->SetSpeed(-0.5f);
							m_RBMotor->SetSpeed(-0.5f);
						}
						else {
							m_LFMotor->SetSpeed(0.f);
							m_LBMotor->SetSpeed(0.f);
							m_RFMotor->SetSpeed(0.f);
							m_RBMotor->SetSpeed(0.f);
							m_lowerIntakeL->SetSpeed(0.5f);
							m_lowerIntakeR->SetSpeed(0.5f);
						}

						break;
					}
					break;
				case 'L':
					switch(autoState) {
					case 0:
						METRO->initPath(path_exchange, PathForward, -20); //who knows
						autoState++; //next case?
						break;
					case 1:
						m_stretchExtend->Set(ControlMode::Position, STRETCH_UP); //stretch extends
						if(advancedAutoDrive()) {
							autoState++; //next case
							autoTimer->Reset(); //reset timer
							autoTimer->Start(); //start timer
						}
						break;
					case 2:
						m_stretchIntake->Set(ControlMode::PercentOutput, 1.f); //run stretch intake
						m_gripperExtend->Set(true); //gripper is extended
						m_gripperRetract->Set(false);
						if(autoTimer->Get() > 1.5f) { //----------------only if doing switch stuff-------------------
							autoState++; //next case
							autoTimer->Reset(); //reset timer
					//		METRO->initPath(h, PathBackward, 90);
						}
						break;
					//--------------------------------------------Switch part------------------------------------------------------------
					case 3:
						m_stretchExtend->Set(ControlMode::Position, STRETCH_TILT); //drop stretch halfway
						m_squareExtend->Set(true); //square is out
						m_squareRetract->Set(false);
						m_lowerIntakeL->SetSpeed(-0.8f); //intake at 80%
						m_lowerIntakeR->SetSpeed(-0.8f);
						advancedAutoDrive();
						if(!m_beamSensorLower->Get()) { //if on then switch state
							autoState++;
							autoTimer->Reset();
			//				METRO->initPath(h, PathForward, 180);
						}
						break;
					case 4:
						m_squareExtend->Set(false); //square is in
						m_squareRetract->Set(true);
						m_gripperExtend->Set(false); //gripper is in
						m_gripperRetract->Set(true);
						m_lowerIntakeL->SetSpeed(0.f); //intake off
						m_lowerIntakeR->SetSpeed(0.f);
						if(autoTimer->Get() < 0.3) { //
							m_LFMotor->SetSpeed(0.5f);
							m_LBMotor->SetSpeed(0.5f);
							m_RFMotor->SetSpeed(-0.5f);
							m_RBMotor->SetSpeed(-0.5f);
						}
						else if(autoTimer->Get() > 0.3 && autoTimer->Get() < 0.6) {
							m_LFMotor->SetSpeed(0.f);
							m_LBMotor->SetSpeed(0.f);
							m_RFMotor->SetSpeed(0.f);
							m_RBMotor->SetSpeed(0.f);
							m_gripperUp->Set(true);
							m_gripperDown->Set(false);
						}
						else
							autoState++;
						break;
					case 5:
						if(autoTimer->Get() < 0.4) {
							m_LFMotor->SetSpeed(0.5f);
							m_LBMotor->SetSpeed(0.5f);
							m_RFMotor->SetSpeed(-0.5f);
							m_RBMotor->SetSpeed(-0.5f);
						}
						else {
							m_LFMotor->SetSpeed(0.f);
							m_LBMotor->SetSpeed(0.f);
							m_RFMotor->SetSpeed(0.f);
							m_RBMotor->SetSpeed(0.f);
							m_lowerIntakeL->SetSpeed(0.5f);
							m_lowerIntakeR->SetSpeed(0.5f);
						}

						break;
					}
					break;
				}
				break;
/*			case 9: //centre switch auto, then exchange
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
						METRO->initPath(path_exchangeLeft, PathForward, 180);
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
						METRO->initPath(path_exchangeRight, PathForward, -180);
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
				break;*/
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

		if(!twoCubeMode) {
			m_gripperDown->Set(true);
			m_gripperUp->Set(false);
			stretchState = 0;
		}
		else
			stretchState = 3;

		gripperTimer->Reset();
		gripperTimer->Start();
		recordTimer->Reset();
		recordTimer->Start();
		stretchTimer->Reset();
		stretchTimer->Start();
		currentGTime = 0.f;
		indicatorState = 0;
		lowerIntakeState = 0;
		joyBlues = false;
		servoHomeAngle = 0.f;
	}

	void TeleopPeriodic() {
/*		switch(driveState) {
		case 1:*/
		switchShotSpeed = limit(-0.5*m_GamepadOp->GetY(XboxController::kRightHand) + 0.5);
		arcadeDrive();
		arcadeShift();
/*			break;
		case 2:
			cheezyDrive();
			cheezyShift();
			cheezyIntake();
			cheezyGripperPneumatics();
			break;
		}*/
		operateIntake();
		operateGripperPneumatics();
		operateConveyor();
#ifndef STRETCH_ENABLED
		applesServo();
#endif
//		calibrateServo();
		indicateCube();
#ifdef STRETCH_ENABLED
		operateStretch();
#endif



		if(autoRecord) {
			printf("RECORDING\n");
			autoFile1 << (std::to_string(m_Joystick->GetY()) + "," + std::to_string(m_Joystick->GetX()) + "," + std::to_string(m_Joystick->GetRawButton(1)) + "," + std::to_string(m_Joystick->GetRawButton(9)) + "," + std::to_string(m_Joystick->GetRawButton(10)) + "," + std::to_string(m_GamepadOp->GetBButton()) + "," + std::to_string(m_GamepadOp->GetTriggerAxis(XboxController::kRightHand)) + "," + std::to_string(m_GamepadOp->GetPOV(0)) + "\n");
		}

		if(m_Joystick->GetRawButton(8) || !autoRecord) {
			autoFile1 << "0,0,0,0,0,0,0,-1";
			autoRecord = false;
			autoFile1.close();
		}
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

/*	void cheezyShift() {
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
	}*/

	bool advancedAutoDrive() {
		float leftSpeed, rightSpeed;
		if(METRO->followPathByEnc(m_leftEncoder->Get(), m_rightEncoder->Get(), nav->GetYaw(), leftSpeed, rightSpeed) == 0) {
			m_LFMotor->SetSpeed(leftSpeed);
			m_LBMotor->SetSpeed(leftSpeed);
			m_RFMotor->SetSpeed(rightSpeed);
			m_RBMotor->SetSpeed(rightSpeed);
		}
		printf("path follow left: %f, right: %f\n", leftSpeed, rightSpeed);
		return METRO->isDone();
	}

	void arcadeDrive() {
		float joyX;
		float joyY;

		joyX = -limit(shiftlib::scale(expo(m_Joystick->GetX(), 5), 0.8)); // Getting the X position from the joystick
		joyY = -limit(shiftlib::scale(expo(m_Joystick->GetY(), 2), 0.8)); // Getting the Y position from the joystick

		m_LFMotor->SetSpeed(-joyY + joyX);
		m_LBMotor->SetSpeed(-joyY + joyX);
		m_RFMotor->SetSpeed(joyY + joyX);
		m_RBMotor->SetSpeed(joyY + joyX);
	}

	bool checkCurrent(float current) {
		return false;//(m_PDP->GetCurrent(15) > current || m_PDP->GetCurrent(14) > current || m_PDP->GetCurrent(13) > current || m_PDP->GetCurrent(12) > current);
	}

/*	void cheezyIntake() {
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
	}*/

	void operateIntake() {
		float intakeFSpeed = limit(m_GamepadOp->GetTriggerAxis(XboxController::kLeftHand));
		float intakeRSpeed = limit(m_GamepadOp->GetTriggerAxis(XboxController::kRightHand));
		float intakeDriveSpeed = limit2(-m_Joystick->GetY(), 0.8, 0.3);

		switch(lowerIntakeState) {
		case 0:
			if(m_gripperDown->Get()) {
/*				m_lowerIntakeL->SetSpeed(intakeFSpeed - 0.7*intakeRSpeed);
				m_lowerIntakeR->SetSpeed(intakeFSpeed - 0.7*intakeRSpeed);*/
				if(m_squareExtend->Get()) {
					m_lowerIntakeL->SetSpeed(-intakeDriveSpeed);
					m_lowerIntakeR->SetSpeed(-intakeDriveSpeed);
				}
				else {
					m_lowerIntakeL->SetSpeed(intakeFSpeed - intakeRSpeed);
					m_lowerIntakeR->SetSpeed(intakeFSpeed - intakeRSpeed);
				}
			}
			else {
				m_lowerIntakeL->SetSpeed(0.5*(intakeFSpeed - intakeRSpeed));
				m_lowerIntakeR->SetSpeed(0.5*(intakeFSpeed - intakeRSpeed));
			}

			if(m_Joystick->GetRawButton(9))
				lowerIntakeState = 10;

			printf("Intake Speed: %f", intakeRSpeed);
			printf("\n");
			break;
		case 10:
			if(m_gripperDown->Get()) {
				m_lowerIntakeL->SetSpeed(1.f);
				m_lowerIntakeR->SetSpeed(1.f);
			}
			else {
				m_lowerIntakeL->SetSpeed(switchShotSpeed);
				m_lowerIntakeR->SetSpeed(switchShotSpeed);
			}

			if(!m_Joystick->GetRawButton(9))
				lowerIntakeState = 0;

			break;
		}
	}

	void operateStretch() {
		if(m_GamepadOp->GetRawButton(GP_R) && m_GamepadOp->GetRawButton(GP_L)) {
			m_stretchExtend->Set(ControlMode::PercentOutput, 0.f);
		}
		else if(m_GamepadOp->GetRawButtonReleased(GP_R) || m_GamepadOp->GetRawButtonReleased(GP_L)) {
			m_stretchExtend->SetSelectedSensorPosition(0, TALON_LOOP_ID, TALON_TIMEOUT);
			stretchState = 0;
		}

		switch(stretchState) {
		case 0:
			if(!(m_GamepadOp->GetRawButton(GP_R) && m_GamepadOp->GetRawButton(GP_L)))
				m_stretchExtend->Set(ControlMode::Position, 0);

			if(m_GamepadOp->GetStartButton())
				stretchState++;
			if(m_GamepadOp->GetPOV(0) == GP_UP) {
				stretchState = 3;
				stretchTimer->Reset();
			}
			break;
		case 1:
			m_stretchExtend->Set(ControlMode::Position, STRETCH_UP);
			if(m_GamepadOp->GetBackButton())
				stretchState++;
			break;
		case 2:
			if(m_stretchExtend->GetSelectedSensorPosition(0) >= 5000)
				m_stretchExtend->Set(ControlMode::PercentOutput, -0.1f);
			else if((m_GamepadOp->GetRawButton(GP_R) && m_GamepadOp->GetRawButton(GP_L)) || m_stretchExtend->GetSelectedSensorPosition(0) < 5000)
				m_stretchExtend->Set(ControlMode::PercentOutput, 0.f);
			break;
		case 3:
			if(stretchTimer->Get() < 2.0)
				m_stretchExtend->Set(ControlMode::Position, STRETCH_TILT);
			else
				m_stretchExtend->Set(ControlMode::PercentOutput, 0.f);

			if(stretchTimer->Get() > 0.7) {
				m_gripperUp->Set(true);
				m_gripperDown->Set(false);
			}

			if(m_GamepadOp->GetPOV(0) == GP_DOWN) {
				stretchTimer->Reset();
				stretchState++;
			}
			break;
		case 4:
			if(stretchTimer->Get() < 2.0)
				m_stretchExtend->Set(ControlMode::Position, STRETCH_TILT);
			else {
				m_stretchExtend->Set(ControlMode::PercentOutput, 0.f);
				stretchState = 0;
			}

			m_gripperUp->Set(false);
			m_gripperDown->Set(true);
		}
	}

	void operateConveyor() {
		float conveyorJoy = limit(m_GamepadOp->GetY(XboxController::kLeftHand));

		if(m_Joystick->GetRawButton(10))
			joyBlues = true;

		switch(conveyorState) {
		case 0:
			if(m_GamepadOp->GetAButton() || m_Joystick->GetRawButton(10))
				conveyorState = 10;

			if(m_Joystick->GetRawButton(11) || m_Joystick->GetRawButton(12))
				m_upperIntakeR->Set(ControlMode::PercentOutput, -0.4f);
			else
				m_upperIntakeR->Set(ControlMode::PercentOutput, 0.f);

			if(m_GamepadOp->GetXButton())
				conveyorState = 5;

			m_upperIntakeL->Set(ControlMode::PercentOutput, 0.f);
			m_conveyor->SetSpeed(conveyorJoy);
#ifdef STRETCH_ENABLED
			m_stretchIntake->Set(ControlMode::PercentOutput, conveyorJoy);
#endif
			break;
		case 5:
			m_conveyor->SetSpeed(CONVEYOR_SPEED);
			m_upperIntakeL->Set(ControlMode::PercentOutput, 0.1f);
			m_upperIntakeR->Set(ControlMode::PercentOutput, 0.1f);

			if(!m_GamepadOp->GetXButton())
				conveyorState = 0;

			break;
		case 10:
			if(m_GamepadOp->GetAButtonReleased() || (!m_Joystick->GetRawButton(10) && joyBlues)) {
				conveyorState = 0;
				joyBlues = false;
			}

			m_conveyor->SetSpeed(CONVEYOR_SPEED);
			m_upperIntakeL->Set(ControlMode::PercentOutput, switchShotSpeed);
			m_upperIntakeR->Set(ControlMode::PercentOutput, switchShotSpeed);
#ifdef STRETCH_ENABLED
			if(stretchState != 1)
				m_stretchIntake->Set(ControlMode::PercentOutput, -switchShotSpeed);
			else
				m_stretchIntake->Set(ControlMode::PercentOutput, switchShotSpeed);
#endif
			break;
		}
	}

	void applesServo() {
#ifdef PRACTICE_BOT
		if(m_Joystick->GetRawButton(11) || m_Joystick->GetRawButton(12) || m_GamepadOp->GetRawButton(GP_R))
			m_tailgateServo->SetAngle(117.f);
		else
			m_tailgateServo->SetAngle(180.f);
#else
		if(m_Joystick->GetRawButton(11) || m_Joystick->GetRawButton(12) || m_GamepadOp->GetRawButton(GP_R))
			m_tailgateServo->SetAngle(84.f);
		else
			m_tailgateServo->SetAngle(156.f);
#endif
	}

	void calibrateServo() {
		if(m_Joystick->GetRawButton(7))
			servoHomeAngle++;
		if(m_Joystick->GetRawButton(8))
			servoHomeAngle--;
		m_tailgateServo->SetAngle(servoHomeAngle);
	}

/*	void cheezyGripperPneumatics() {
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
	}*/

	void operateGripperPneumatics() {
		if(m_GamepadOp->GetBButton()) {
			m_gripperExtend->Set(true);
			m_gripperRetract->Set(false);
			if(gripperTimer->Get() > 0.5f) {
				m_squareExtend->Set(true);
				m_squareRetract->Set(false);
			}
		}
		else {
			m_gripperExtend->Set(false);
			m_gripperRetract->Set(true);
			m_squareExtend->Set(false);
			m_squareRetract->Set(true);
			gripperTimer->Reset();
		}

#ifndef STRETCH_ENABLED
		if(m_GamepadOp->GetPOV(0) == GP_UP) {
			m_gripperUp->Set(true);
			m_gripperDown->Set(false);
		}
		else if(m_GamepadOp->GetPOV(0) == GP_DOWN) {
			m_gripperUp->Set(false);
			m_gripperDown->Set(true);
		}
#endif
	}

/*	void cheezyDrive() {
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
	}*/

	void indicateCube() {
		if(!m_beamSensorLower->Get())
			m_mangoRingLight->Set(Relay::kForward);
		else
			m_mangoRingLight->Set(Relay::kOff);
	}

// #.f is used to tell the computer that the number here is not a whole number(1).
	float limit(float n) { // The range and domain are equal to 1.f and -1.f
		if(n > 1.f)
			return 1.f; // If "n" is greater then 1.f then return positive 1.f
		if(n < -1.f)
			return -1.f; // If "n" is less then -1.f then return negative -1.f
		return n;
	}

	float limit2(float n, float max, float min) {
		if(n > max)
			return max;
		if(n < min)
			return min;
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
