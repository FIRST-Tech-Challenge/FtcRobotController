package org.firstinspires.ftc.teamcode.util;

/**
 * Here, store the names for each of the motors in the hardware map,
 * store the pid constants for each of the loops,
 * and store any other constants that are used in the code.
 */
public class Constants
{
    public static String imu = "imu";
    public static String frontWebcamera = "Webcam 1";
    public static double TICKS_TO_INCHES = 1;
    public static String motorPositionFolder = "/MOTORS";
    public static String teamConfigFolder = "/MYTEAM";
    public static String cameraConfigFolder = "/CAMERA";
    public static int CameraViewWidth = 640;
    public static int CameraViewHeight = 360;

    // Arm Constants
    public static int armCurrentPosition = 0;
    public static int armPickupPosition = 10;
    public static int armDropPosition = 125;
    public static int armFrontScorePosition = 1010;
    public static int armBackScorePosition = 5130;//5240; //4710;
    public static int armInitPosition = 25;
    public static int armPropPosition = 100;
    public static int armParkPosition = 10;
    public static int armStack1Position = 228;
    public static int armStack2Position = 200;
    public static int armClimbPrepPosition = 2530;
    public static int armClimbPosition = 500;
    public static int armStowPosition = 10;
    public static int armSlowDrivePosition = 2680;
    public static int armRotateScale = 200;
    public static int armBackScoreTeleopPosition = 4850;;//4710;

    // Wrist Constants
    public static int wristCurrentPosition = 0;
    public static int wristPickupPosition = -2130;//-2230;
    public static int wristFrontScorePosition = -1680;
    public static int wristBackScorePosition = -400;//-585;//-353;
    public static int wristPropPosition = -2000;
    public static int wristInitPosition = 0;
    public static int wristStowPosition = -25;
    public static int wristParkPosition = 0;
    public static int wristDropPosition = -2100;
    public static int wristStack2Position = -2232;
    public static int wristStack1Position = -2232;
    public static int wristClimbPrepPosition = -2222;
    public static int wristClimbPosition = -1290;
    public static int wristBackScoreTeleopPosition = -190;

    public static int wristRotateScale = 50;

    // PID values for wrist and arm
    public static double wristkP = .005;
    public static double wristkD = 0;
    public static double wristkI = 0;
    public static double wristkF = 0;

    public static double armkP = 0.01;
    public static double armkD = 0.00001;
    public static double armkI = 0.0001;

    public static double wristPower = 0;
    public static double armPower = 0;

    // Gripper positions
    public static double gripperLeftClosedPosition = 0.8;
    public static double gripperLeftOpenPosition = 0.5;

    public static double gripperRightClosedPosition = 0.25;
    public static double gripperRightOpenPosition = 0.55;

    // Launcher ps
    public static double launcherLaunchPosition = 0.5;
    public static double launcherHoldPosition = 0.675;

    public static double MAX_AUTO_SPEED = 0.5;
    public static double MAX_AUTO_STRAFE= 0.5;
    public static double MAX_AUTO_TURN  = 0.3;
}
