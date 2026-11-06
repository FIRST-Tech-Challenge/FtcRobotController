package org.firstinspires.ftc.teamcode;

/****
 * robotGlobalSettings defines all of the hardware settings for our robot. It also contains
 * auton configuration parameters.
 */
public class RobotGlobalSettings {

    public enum FieldSide {Blue, Red}

    // ====================== Competition settings -> can override in specific teleop/auton
    static FieldSide competitionSide = FieldSide.Blue;

    // ====================== Robot dimensions
    public static double RobotLength_inch = 18;
    public static double RobotWidth_inch = 16;

    // ====================== Dimensions of field objects =========================
    public static double OneTileLength_inch = 23 + (3.0/4.0);

    // ====================== Data tracing
    static boolean dataTracingIsActive = false;

    // ====================== Camera settings
    /*static robotCameraSettings cameraSettings = new robotCameraSettings();

    // ====================== Intake settings
    static robotIntakeSettings IntakeSettings = new robotIntakeSettings();

    // ====================== Warehouse side auton settings
    static robotAutonWarehouseSettings autonRedWarehouseSettings = new robotAutonWarehouseSettings();
    static robotAutonWarehouseSettings autonBlueWarehouseSettings = new robotAutonWarehouseSettings();
    static robotAutonWarehouseSettings autonWarehouseSettings; // gets selected when the field side is chosen
*/
    // ====================== Rev Hex HD Motor 2240 counts per rotation
    public static double COUNTS_PER_MOTOR_REV =      537.6; ///< The number of encoder counts per output shaft revolution for 5202-0002-0019 GoBilda motor
    public static double DRIVE_GEAR_REDUCTION =       1; ///< The gear ration outside of the motor assembly
    public static double WHEEL_DIAMETER_INCHES =     3.937; ///< The diameter of the drive wheels
    public static double COUNTS_PER_INCH =(COUNTS_PER_MOTOR_REV / DRIVE_GEAR_REDUCTION)/(WHEEL_DIAMETER_INCHES*3.1415);

    public static double ImuCorrectionFactor = 0.04; ///< The gain value for IMU based corrections to moving in straight line.
    ///< The value is set by experiment.

    // ====================== Robot linear speed ramp up settings
    public static double RampUpTime = 0.35; ///< Ramp time in seconds to reach the commanded speed. The shorter the time, the
    ///< larger the chance of wheel slip.

    // ====================== Robot linear speed ramp down settings
    public static int EncoderCountRampDownThreshold = (int)(20*COUNTS_PER_INCH);
    public static double topWheelAngularVelocity_radPerSec = 15; ///< The highest wheel ang velocity that the robot reaches at full power
    public static double LinearRampDownMinimumPower = 0.2; ///< Do not ramp down power to a smaller value so as to not get stuck due to friction

    // ====================== Settings for rotating
    public static double rotationRampTimeInSec = 0.35;
    public static double powerRampdownStartAngle = 45;
    public static double RotationRampDownMinimumPower = 0.2; // Do not ramp down speed to a smaller value so as to not get stuck due to friction

    // ====================== Data server settings
    public static String dataServerHostIP = "192.168.43.17";
    public static int dataServerHostPort = 9876;

    // ====================== Fourbar Settings
    public static double fourbar_intake_position = 1.65;
    public static double fourbar_aboveFreight_position = 1.3;
    public static double fourbar_bottom_position = 1.32;
    public static double fourbar_middle_position = 1.10; //from 1.1 12/8/2021, 1.18 on 12/9
    public static double fourbar_top_position = 0.83;
    public static double fourbar_auton_top_position = 0.8;
    public static double fourbar_topLimit_position = 0.74;
    public static double fourbar_positionDeadband = 0.06;

    public static double fourbar_pGain_DownPowerCtrl = 0.5;
    public static double fourbar_dGain_DownPowerCtrl = 1/29000.0;
    public static double fourbar_pGain_UpPowerCtrl = 5;

    public static double fourbar_intake_positionEncoder = 0;
    public static double fourbar_aboveFreight_positionEncoder = -0.270;
    public static double fourbar_bottom_positionEncoder = -0.314;
    public static double fourbar_middle_positionEncoder = -0.800;//-0.772;
    public static double fourbar_top_positionEncoder = -1.603;
    public static double fourbar_auton_top_positionEncoder = -1.650;
    public static double fourbar_topLimit_positionEncoder = -1.950;
    public static double fourbar_positionEncoderDeadband = 0.02;//0.06;

    public static double fourbar_pGain_DownPowerCtrl_Encoder = 1;
    public static double fourbar_pGain_UpPowerCtrl_Encoder = 4.5;

    // ====================== Turntable contact detection
    public static double turntable_onContactAmpsThreshold = 1.13;//0.45;
    public static double turntable_speedDegPerSec = 40;//45


    public static void initialize(){
        //============== Red side warehouse settings
       /* autonRedWarehouseSettings.initialDeliveryIMUcorrection = 0.022;
        autonRedWarehouseSettings.toWarehouseIMUcorrection = 0.018;
        autonRedWarehouseSettings.fromWarehouseIMUcorrection = 0.018;
        autonRedWarehouseSettings.intialStraffDist_in = 0;
        autonRedWarehouseSettings.moveOffTheWallDistance_in = 3;
        autonRedWarehouseSettings.driveAngleToShippingHub_deg = 23;
        autonRedWarehouseSettings.firstDriveDistanceToShippingHub = 20;
        autonRedWarehouseSettings.driveAngleToWarehouse_deg = -90;
        autonRedWarehouseSettings.biasDirectionToWarehouse = robotAutonWarehouseSettings.BiasDirection.twoOclock;
        autonRedWarehouseSettings.biasDirectionFromWarehouse = robotAutonWarehouseSettings.BiasDirection.tenOClock;
        autonRedWarehouseSettings.biasAmountToWarehouse = 0.15;
        autonRedWarehouseSettings.driveIntoTheWarehouse_in = 15;
        autonRedWarehouseSettings.driveInTheWarehouse_in = robotGlobalSettings.OneTileLength_inch * 0.9;
        autonRedWarehouseSettings.driveOutOfTheWarehouse_in = 14;
        autonRedWarehouseSettings.driveDistanceFromShippingHub = 7.5;
        autonRedWarehouseSettings.driveRatioFromShippingHubFirstPart = 0.5;
        autonRedWarehouseSettings.driveDistanceToShippingHub = 23;
        autonRedWarehouseSettings.distanceSensorTriggerLevel = 175;

        //============== Blue side warehouse settings
        autonBlueWarehouseSettings.initialDeliveryIMUcorrection = 0.022;
        autonBlueWarehouseSettings.toWarehouseIMUcorrection = 0.018;
        autonBlueWarehouseSettings.fromWarehouseIMUcorrection = 0.022;
        autonBlueWarehouseSettings.intialStraffDist_in = 19;
        autonBlueWarehouseSettings.moveOffTheWallDistance_in = 2;
        autonBlueWarehouseSettings.driveAngleToShippingHub_deg = -25;
        autonBlueWarehouseSettings.firstDriveDistanceToShippingHub = 26;
        autonBlueWarehouseSettings.driveAngleToWarehouse_deg = 90;
        autonBlueWarehouseSettings.biasDirectionToWarehouse = robotAutonWarehouseSettings.BiasDirection.tenOClock;
        autonBlueWarehouseSettings.biasDirectionFromWarehouse = robotAutonWarehouseSettings.BiasDirection.twoOclock;
        autonBlueWarehouseSettings.biasAmountToWarehouse = 0.16;
        autonBlueWarehouseSettings.driveIntoTheWarehouse_in = 6;
        autonBlueWarehouseSettings.driveInTheWarehouse_in = robotGlobalSettings.OneTileLength_inch * 0.9;
        autonBlueWarehouseSettings.driveOutOfTheWarehouse_in = 14;
        autonBlueWarehouseSettings.driveDistanceFromShippingHub = robotGlobalSettings.OneTileLength_inch * 1.00;
        autonBlueWarehouseSettings.driveRatioFromShippingHubFirstPart = 0.2;
        autonBlueWarehouseSettings.driveDistanceToShippingHub = robotGlobalSettings.OneTileLength_inch * 1.07;
        autonBlueWarehouseSettings.distanceSensorTriggerLevel = 175;
*/
    }

    static double lastImuCorrectionFactor;
    public static void setImuCorrectionFactor(double value) {
        lastImuCorrectionFactor = ImuCorrectionFactor;
        ImuCorrectionFactor = value;
    }

    public static void restoreImuCorrectionFactor(){
        ImuCorrectionFactor = lastImuCorrectionFactor;
    }
}
