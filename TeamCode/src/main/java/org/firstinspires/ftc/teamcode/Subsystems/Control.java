package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import static java.lang.Thread.sleep;

/**
 * Control subsystem for controlling arms and claws
 * Created by AndrewC on 1/17/2020
 */
public class Control extends Subsystem {
    // device declaration
    private HardwareMap hardwareMap;
    private LinearOpMode opMode;

    //DC Motors
    private DcMotorEx intake;
    private DcMotorEx launch1;
    private DcMotorEx launch2a;
    private DcMotorEx launch2b;

    //Servos
    public Servo elevatorR;
    public Servo elevatorL;
    private Servo wobbleClaw;
    private Servo wobbleGoalArm;
    private Servo intakeToElevatorL;
    private Servo intakeToElevatorR;
    private Servo launcherFeederL;
    private Servo launcherFeederR;


    //Sensors
    private BNO055IMU imu;

    // define physical/structural constants

    /**
     * The TILT_TABLE is a lookup table for mapping the main arm angle to the arm tilt motor tick
     * The TILT_TABLE_SIZE records the number of entries in the TILT_TABLE
     * The TILT_TABLE consists of TILT_TABLE_SIZE pairs of data. Each pair is (tilt angle, arm tilt motor tick).
     */
    private static final int        TILT_TABLE_SIZE                     = 83;
    private static final double[]   TILT_TABLE = {
            0.0, 0, 0.6, 88, 1.5, 131, 2.2, 160, 2.9, 200, 3.4, 223, 4.1, 257, 5.0, 301, 5.8, 350, 6.7, 392,
            7.3, 432, 8.2, 479, 9.1, 535, 9.7, 571, 10.5, 617, 11.2, 658, 11.7, 686, 12.2, 716, 12.7, 748, 13.2, 781,
            13.6, 810, 14.1, 838, 14.7, 880, 15.2, 918, 15.8, 951, 16.1, 972, 16.9, 1023, 17.9, 1093, 18.5, 1131, 18.9, 1158,
            19.5, 1205, 20.2, 1252, 20.9, 1300, 21.3, 1333, 21.9, 1376, 22.7, 1428, 23.3, 1473, 23.9, 1518, 25.0, 1602, 25.9, 1666,
            26.9, 1736, 27.8, 1801, 28.5, 1854, 29.5, 1932, 30.3, 1997, 31.2, 2067, 32.0, 2120, 33.0, 2199, 34.0, 2269, 34.7, 2336,
            35.5, 2391, 36.5, 2470, 37.2, 2534, 38.2, 2608, 38.9, 2661, 40.1, 2750, 40.8, 2812, 41.4, 2862, 42.2, 2923, 43.1, 3000,
            44.0, 3060, 45.1, 3152, 45.9, 3210, 46.7, 3284, 47.8, 3372, 48.7, 3444, 49.4, 3501, 50.5, 3584, 51.0, 3630, 51.9, 3700,
            53.3, 3804, 54.4, 3893, 55.5, 3987, 57.0, 4106, 58.1, 4186, 59.6, 4301, 61.0, 4414, 62.0, 4491, 63.0, 4564, 63.7, 4615,
            64.7, 4688, 65.9, 4771, 67.5, 4889 };

    /**
     * The CLAW_ARM_TILT_TABLE is a lookup table for mapping the claw arm angle to the claw arm servo
     * The CLAW_ARM_TILT_TABLE_SIZE records the number of entries in the CLAW_ARM_TILT_TABLE
     * The CLAW_ARM_TILT_TABLE consists of CLAW_ARM_TILT_TABLE_SIZE pairs of data. Each pair is (tilt angle, arm tilt servo).
     */
    private static final int        CLAW_ARM_TILT_TABLE_SIZE                     = 24;
    private static final double[]   CLAW_ARM_TILT_TABLE = {
            -183.0, 0.01, -180.0, 0.04, -70.2, 0.416, -30.8, 0.563, -12.2, 0.634, -5.8, 0.658, -2.9, 0.679, 0.2, 0.692, 4.6, 0.703, 6.1, 0.716,
            11.0, 0.736, 15.2, 0.743, 18.2, 0.763, 23.2, 0.782, 29.0, 0.804, 33.9, 0.825, 40.8, 0.851, 46.6, 0.879, 51.0, 0.894, 57.4, 0.921,
            62.5, 0.941, 70.1, 0.971, 75.6, 0.993, 77.6, 1.0 };

    //DO WITH ENCODERS
    private static final double     TICKS_PER_MOTOR_REV_40          = 1120;    // AM Orbital 20 motor
    private static final double     RPM_MAX_NEVERREST_40            = 160;
    private static final double     ANGULAR_V_MAX_NEVERREST_40      = (TICKS_PER_MOTOR_REV_40 * RPM_MAX_NEVERREST_40) / 60.0;

    private static final double     WINCH_DIAMETER_INCH                 = 1.244;  //inch original measurement
    private static final double     WINCH_DIAMETER_MM                   = WINCH_DIAMETER_INCH * 2.54 * 10.0; //milimeters
    private static final double     WINCH_RADIUS_MM                     = WINCH_DIAMETER_MM / 2.0;
    private static final double     WINCH_CIRCUMFERENCE_MM              = WINCH_RADIUS_MM * 2.0 * Math.PI;
    private static final double     MOTOR_TICK_PER_REV_NEVERREST40      = 1120.0;
    private static final double     MOTOR_TICK_PER_REV_YELLOJACKET223   = 753.2;
    private static final double     REV_PER_MIN_YELLOJACKET223          = 223.0;
    private static final double     MOTOR_TICK_PER_REV_YELLOJACKET1620   = 103.6;
    private static final double     REV_PER_MIN_YELLOJACKET1620          = 1620.0;
    private static final double     WINCH_MAX_SPEED_MM_PER_SEC          = (RPM_MAX_NEVERREST_40 * WINCH_DIAMETER_MM * Math.PI) / 60.0;
    private static final double     WINCH_MAX_SPEED_TICK_PER_SEC        = (MOTOR_TICK_PER_REV_NEVERREST40 * RPM_MAX_NEVERREST_40) / 60.0;
    private static final double     TILT_MAX_SPEED_TICK_PER_SEC         = (MOTOR_TICK_PER_REV_YELLOJACKET1620 * REV_PER_MIN_YELLOJACKET1620) / 60.0;
    //    private static final double     TILT_TICK_PER_90_DEGREE             = 2510.0;
    private static final double     WINCH_MM_PER_TICK                   = WINCH_CIRCUMFERENCE_MM / MOTOR_TICK_PER_REV_NEVERREST40;

    private static final double     MAINARM_INIT_LENGTH          = 371.0;    // main arm length before extending (mm)
    private static final double     MAINARM_STACK_HEIGHT         = 123.0;    // main arm vertical offset (mm)

    private static final double     MAINARM_LENGTH_TICK_MAX         = 8900.0;    // main arm tick max


    // THESE NEXT VALUES NEED TO BE SET LATER - wobble goal for ultimategoal
    private static final double     WB_ARM_POS_ANGLE                  = 0; // most positive angle
    private static final double     WB_ARM_POS_VALUE                  = 0; // servo setting at most positive angle
    private static final double     WB_CLAW_POS_OPEN_WIDE              = 0;
    private static final double     WB_CLAW_POS_OPEN                  = 0;
    private static final double     WB_CLAW_POS_CLOSED_STONE          = 0;
    private static final double     WB_CLAW_POS_CLOSED                = 0;

    private static final double     WOBBLE_GOAL_ARM_DOWN         = 0.0;
    private static final double     WOBBLE_GOAL_ARM_LOW          = 0.165;
    private static final double     WOBBLE_GOAL_ARM_MED          = 0.300;
    private static final double     WOBBLE_GOAL_ARM_HIGH          = 0.440;



    private static final double     WOBBLE_GOAL_CLAW_OPEN        = 0.459;
    private static final double     WOBBLE_GOAL_CLAW_OPEN_WIDE   = 0.579;
    private static final double     WOBBLE_GOAL_CLAW_CLOSED      = 0.214;

    private static final double     INTAKE_TO_ELEVATOR_R_OPEN    = 0.435;
    private static final double     INTAKE_TO_ELEVATOR_R_CLOSE   = 0.089;
    private static final double     INTAKE_TO_ELEVATOR_L_OPEN    = 0.56;
    private static final double     INTAKE_TO_ELEVATOR_L_CLOSE   = 0.905;

    private static final double     LAUNCHER_FEEDER_R_LAUNCH     = 0.470;
    private static final double     LAUNCHER_FEEDER_R_REST       = 0.580;
    private static final double     LAUNCHER_FEEDER_L_LAUNCH     = 0.700;
    private static final double     LAUNCHER_FEEDER_L_REST       = 0.385;

    private static final double     ELEVATOR_BOTTOM_POS_R         = 0.815;
    private static final double     ELEVATOR_BOTTOM_POS_L         = 0.105;

    private static final double     ELEVATOR_1RING_POS_R          = 0.153;
    private static final double     ELEVATOR_1RING_POS_L          = 0.823;

    private static final double     ELEVATOR_2RING_POS_R          = 0.274;
    private static final double     ELEVATOR_2RING_POS_L          = 0.704;

    private static final double     ELEVATOR_3RING_POS_R          = 0.389;
    private static final double     ELEVATOR_3RING_POS_L          = 0.570;

    private static final double     ELEVATOR_1RING_LAUNCH_POS_R          = 0.313;
    private static final double     ELEVATOR_1RING_LAUNCH_POS_L          = 0.673;

    private static final double     ELEVATOR_2RING_LAUNCH_POS_R          = 0.394;
    private static final double     ELEVATOR_2RING_LAUNCH_POS_L          = 0.588;

    private static final double     ELEVATOR_3RING_LAUNCH_POS_R          = 0.529;
    private static final double     ELEVATOR_3RING_LAUNCH_POS_L          = 0.444 ;

    private static final double     LAUNCHER_ANG_PER_SEC_LIMIT = 722.0*2.0;



    // define variables
    private double mainArmAngle = 0.0;
    private double mainArmTargetAngle = 0.0;
    private double mainArmAngleTick = 0.0;
    private double mainArmLength = 0.0;
    private double mainArmLengthTick = 0.0;
    private boolean mainClawArmTrackingMode = false;
    private double ClawRotationAngle = 0.0;

    private int elevatorStage = 0;
    /**
     * 0 = BOTTOM
     * 1 = 3 Rings position
     * 2 = 2 Rings position
     * 3 = 1 Ring position
     */

    private double launcherRPMLimit = 1800.0;
    private double launcherCurrentRPM;
    private double launcherTargetRPM;
    private double launcherKp = 0.005;
    private double launcherKi = 0.000003;
    private double launcherKd = 0.01;


    //    public Control(DcMotorEx intake, DcMotorEx launch1, DcMotorEx launch2, BNO055IMU imu, LinearOpMode opMode, ElapsedTime timer, ) {
    public Control(DcMotorEx intake, DcMotorEx launch1, DcMotorEx launch2a, DcMotorEx launch2b, BNO055IMU imu, LinearOpMode opMode, ElapsedTime timer, Servo wobbleClaw, Servo wobbleGoalArm) {

        // store device information locally
        this.wobbleClaw = wobbleClaw;
        this.wobbleGoalArm = wobbleGoalArm;
        this.intake = intake;
        this.launch1 = launch1;
        this.launch2a = launch2a;
        this.launch2b = launch2b;
        this.opMode = opMode;
        this.hardwareMap = opMode.hardwareMap;
        this.imu = imu;
        this.timer = timer;
        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // initialize main arm parameters
        mainClawArmTrackingMode = false;
    }

    public Control(DcMotorEx intake, DcMotorEx launch1, DcMotorEx launch2a, DcMotorEx launch2b, BNO055IMU imu, LinearOpMode opMode, ElapsedTime timer,
                   Servo wobbleClaw, Servo wobbleGoalArm, Servo intakeToElevatorR, Servo intakeToElevatorL, Servo launcherFeederR, Servo launcherFeederL, Servo elevatorR, Servo elevatorL) {

        // store device information locally
        this.wobbleClaw = wobbleClaw;
        this.wobbleGoalArm = wobbleGoalArm;
        this.intakeToElevatorR = intakeToElevatorR;
        this.intakeToElevatorL = intakeToElevatorL;
        this.launcherFeederR = launcherFeederR;
        this.launcherFeederL = launcherFeederL;
        this.elevatorR = elevatorR;
        this.elevatorL = elevatorL;

        this.intake = intake;
        this.launch1 = launch1;
        this.launch2a = launch2a;
        this.launch2b = launch2b;
        this.opMode = opMode;
        this.hardwareMap = opMode.hardwareMap;
        this.imu = imu;
        this.timer = timer;
        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // initialize main arm parameters
        mainClawArmTrackingMode = false;
    }
    public void initServo(){
        openIntakeToElevator();
        restLauncherFeeder();
        moveElevatorToBottom();
        moveWobbleGoalArmDown();
        openWideWobbleGoalClaw();
    }

    /**
     * Sets all drive motors to specified zero power behavior
     */
    private void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior mode) {
        intake.setZeroPowerBehavior(mode);
        launch1.setZeroPowerBehavior(mode);
        launch2a.setZeroPowerBehavior(mode);
        launch2b.setZeroPowerBehavior(mode);
    }

    public double getWinchMaxSpeedMMpSec(){
        return WINCH_MAX_SPEED_MM_PER_SEC;
    }
    public double getWinchMaxSpeedTickPerSec(){
        return WINCH_MAX_SPEED_TICK_PER_SEC;
    }
    public double getTiltMaxSpeedTickPerSec(){
        return TILT_MAX_SPEED_TICK_PER_SEC;
    }
    public double getAngularVMaxNeverrest40(){
        return ANGULAR_V_MAX_NEVERREST_40;
    }
    public double getMotorTickPerRevYellojacket223(){
        return MOTOR_TICK_PER_REV_YELLOJACKET223;
    }

    public void openWobbleGoalClaw() {
        wobbleClaw.setPosition(WOBBLE_GOAL_CLAW_OPEN);
    }
    public void closeWobbleGoalClaw(){
        wobbleClaw.setPosition(WOBBLE_GOAL_CLAW_CLOSED);
    }
    public void openWideWobbleGoalClaw(){wobbleClaw.setPosition(WOBBLE_GOAL_CLAW_OPEN_WIDE);};

    public void moveWobbleGoalArmDown(){
        wobbleGoalArm.setPosition(WOBBLE_GOAL_ARM_DOWN);
    }

    public void raiseWobbleGoalArmLow(){
        wobbleGoalArm.setPosition(WOBBLE_GOAL_ARM_LOW);
    }

    public void raiseWobbleGoalArmMed(){
        wobbleGoalArm.setPosition(WOBBLE_GOAL_ARM_MED);
    }

    public void raiseWobbleGoalArmHigh(){
        wobbleGoalArm.setPosition(WOBBLE_GOAL_ARM_HIGH);
    }

    public void deployWobble() {
        wobbleGoalArm.setPosition(0.934);
    }

//    public double getWobbleArmTargetAngle() {
//        return mainArmTargetAngle;
//    }
//    public double wobbleGoalArmAngleToPos(double angle){
//        int lowerIndex, upperIndex;
//        int i = 1;
//        double servoTarget;
//        while ((i < CLAW_ARM_TILT_TABLE_SIZE) && (CLAW_ARM_TILT_TABLE[i*2] < angle)) {
//            ++i;
//        }
//        upperIndex = i;
//        lowerIndex = i-1;
//        servoTarget = CLAW_ARM_TILT_TABLE[lowerIndex*2+1] +
//                (CLAW_ARM_TILT_TABLE[upperIndex*2+1]-CLAW_ARM_TILT_TABLE[lowerIndex*2+1])*(angle-CLAW_ARM_TILT_TABLE[lowerIndex*2])
//                        /(CLAW_ARM_TILT_TABLE[upperIndex*2]-CLAW_ARM_TILT_TABLE[lowerIndex*2]);
//        return servoTarget;
//    }
//    public void setWobbleAngle(double angle){
//        wobbleGoalArm.setPosition(this.wobbleGoalArmAngleToPos(angle));
//    }
//    public void retractWobbleClaw(){
//        setWobbleAngle(-180);
//    }

    public void closeIntakeToElevator(){
        intakeToElevatorR.setPosition(INTAKE_TO_ELEVATOR_R_CLOSE);
        intakeToElevatorL.setPosition(INTAKE_TO_ELEVATOR_L_CLOSE);
    }

    public void openIntakeToElevator(){
        intakeToElevatorR.setPosition(INTAKE_TO_ELEVATOR_R_OPEN);
        intakeToElevatorL.setPosition(INTAKE_TO_ELEVATOR_L_OPEN);
    }

    public void restLauncherFeeder(){
        launcherFeederR.setPosition(LAUNCHER_FEEDER_R_REST);
        launcherFeederL.setPosition(LAUNCHER_FEEDER_L_REST);
    }

    public void launchLauncherFeeder(){
        launcherFeederR.setPosition(LAUNCHER_FEEDER_R_LAUNCH);
        launcherFeederL.setPosition(LAUNCHER_FEEDER_L_LAUNCH);
    }

    public void launchLauncherFeeder1(){
        launcherFeederR.setPosition(LAUNCHER_FEEDER_R_LAUNCH);
    }

    public void launchLauncherFeeder2(){
        launcherFeederL.setPosition(LAUNCHER_FEEDER_L_LAUNCH);
    }

    public void setIntake(boolean status){
        if(status){
            intake.setPower(1.0);
        }
        else {
            intake.setPower(0.0);
        }
    }

    public void setIntakeReverse(boolean status){
        if(status){
            intake.setPower(-0.8);
        }
        else {
            intake.setPower(0.0);
        }
    }

    public void setLaunch(boolean status){
        if (status){
            launch1.setPower(-1.0);
            launch2a.setPower(-1.0);
            launch2b.setPower(-1.0);
        }
        else{
            launch1.setPower(0.0);
            launch2a.setPower(0.0);
            launch2b.setPower(0.0);
        }
    }

    public void setLaunchPower(double launchPower){
        launch1.setPower(-launchPower);
        launch2a.setPower(-launchPower);
        launch2b.setPower(-launchPower);
    }

    public void setLaunchVelocity(double angularRate){
        launch1.setVelocity(angularRate);
        launch2a.setVelocity(angularRate);
        launch2b.setVelocity(angularRate);
    }

    public void setLauncherRPMLimit(double launcherRPM) { launcherRPMLimit = launcherRPM; }

    public double getLauncherRPMLimit() { return launcherRPMLimit; }

    public void setLauncherCurrentRPM(double launcherRPM) { launcherCurrentRPM = launcherRPM; }

    public double getLauncherCurrentRPM() { return launcherCurrentRPM; }

    public void setLauncherTargetRPM(double launcherRPM) {
        launcherTargetRPM = launcherRPM;
    }

    public double getLauncherTargetRPM() { return launcherTargetRPM; }

    public void setLauncherKp(double kp) { launcherKp = kp; }

    public double getLauncherKp() { return launcherKp; }

    public void setLauncherKi(double ki) { launcherKi = ki; }

    public double getLauncherKi() { return launcherKi; }

    public void setLauncherKd(double kd) { launcherKd = kd; }

    public double getLauncherKd() { return launcherKd; }

    public int getElevatorStage(){
        return elevatorStage;
    }

    public void moveElevator(int x){
        int newElevatorStage = elevatorStage + x;
        if(newElevatorStage > 3){
            newElevatorStage = 3;
        }
        if(newElevatorStage < 0){
            newElevatorStage = 0;
        }
        elevatorStage = newElevatorStage;
        switch(elevatorStage){
            case 0:
                elevatorR.setPosition(ELEVATOR_BOTTOM_POS_R);
                elevatorL.setPosition(ELEVATOR_BOTTOM_POS_L);
                break;
            case 1:
                elevatorR.setPosition(ELEVATOR_3RING_POS_R);
                elevatorL.setPosition(ELEVATOR_3RING_POS_L);
                break;
            case 2:
                elevatorR.setPosition(ELEVATOR_2RING_POS_R);
                elevatorL.setPosition(ELEVATOR_2RING_POS_L);
                break;
            case 3:
                elevatorR.setPosition(ELEVATOR_1RING_POS_R);
                elevatorL.setPosition(ELEVATOR_1RING_POS_L);
                break;
        }
    }

    // lower elevator to launch position to get better angle when ring contacts with fly wheel
    public void moveElevatorLaunch(){
        switch(elevatorStage){
            case 0:
                elevatorR.setPosition(ELEVATOR_BOTTOM_POS_R);
                elevatorL.setPosition(ELEVATOR_BOTTOM_POS_L);
                break;
            case 1:
                elevatorR.setPosition(ELEVATOR_3RING_LAUNCH_POS_R);
                elevatorL.setPosition(ELEVATOR_3RING_LAUNCH_POS_L);
                break;
            case 2:
                elevatorR.setPosition(ELEVATOR_2RING_LAUNCH_POS_R);
                elevatorL.setPosition(ELEVATOR_2RING_LAUNCH_POS_L);
                break;
            case 3:
                elevatorR.setPosition(ELEVATOR_1RING_LAUNCH_POS_R);
                elevatorL.setPosition(ELEVATOR_1RING_LAUNCH_POS_L);
                break;
        }
    }

    public void moveElevatorToBottom(){
        elevatorStage = 0;
        elevatorR.setPosition(ELEVATOR_BOTTOM_POS_R);
        elevatorL.setPosition(ELEVATOR_BOTTOM_POS_L);
    }

    public void launchOneRing() throws InterruptedException {
        launchLauncherFeeder1();
        sleep(200);
        moveElevatorLaunch();
        sleep(100);
        launchLauncherFeeder2();
        closeIntakeToElevator();
        sleep(500);
        restLauncherFeeder();
        openIntakeToElevator();
        sleep(355);
        moveElevator(1);
    }

    public void modifyServo(Servo servo, double value) {
        double currentValue = servo.getPosition();
        currentValue = currentValue + value;
        if (currentValue > 1.0) currentValue = 1.0;
        if (currentValue < 0.0) currentValue = 0.0;
        servo.setPosition(currentValue);
    }
    //complete later. need to take some robot position (absolute)/angle values as input and calculate how much to rotate the turret
    public double autoRotateTurret() {
        double angle = 0;
        return angle;
    }

    public double tickPerSecTORPM(double angPerSec){
        return ((angPerSec / 28.0) * 60.0);
    }

    public double getLauncherAngPerSecLimit(){
        return LAUNCHER_ANG_PER_SEC_LIMIT;
    }
}
