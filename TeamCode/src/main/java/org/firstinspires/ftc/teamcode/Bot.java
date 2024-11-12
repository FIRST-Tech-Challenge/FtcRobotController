package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Bot {

    //OpMode Declaration
    private LinearOpMode opMode;

    //Motor Declaration
    private DcMotor leftMotorFront;
    private DcMotor rightMotorFront;
    private DcMotor leftMotorBack;
    private DcMotor rightMotorBack;

    private DcMotor rightLift;
    private DcMotor leftLift;

    private CRServo leftIntake;
    private CRServo rightIntake;

    private HardwareMap hwMap = null;

    //extend and pivot
    private DcMotor extendArmMotor;
    private DcMotor armPivotMotor;

    private CRServo rightPushoff;
    private CRServo leftPushoff;


    //Statistics for measurements
    public static final int MAX_EXT = -2648;
    public static final double MIN_EXTEND = 0;

    public static final int LEFT_LIFT_MAX = 7255;
    public static final int LEFT_LIFT_MIN = -101;
    public static final int RIGHT_LIFT_MAX = 7302;
    public static final int RIGHT_LIFT_MIN = -10;
    public static final int MAX_PIVOT = 2560;
    public static final int MIN_PIVOT = -180;


    //Drive Encoder Stats
    private static final double COUNTS_PER_MOTOR_REV = 537.7;    // GoBilda 5203 motor encoder res
    private static final double DRIVE_GEAR_REDUCTION = 1.0;     // 1:1
    private static final double WHEEL_DIAMETER_INCHES = 4.09449;     // For figuring circumference
    private static final double CIRCUMFERENCE = (WHEEL_DIAMETER_INCHES * Math.PI);
    private static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / CIRCUMFERENCE;
    private static final double DISTANCE_PER_ENCODER =  CIRCUMFERENCE/COUNTS_PER_MOTOR_REV;


    private static final double MAX_DISTANCE = 25.5;
    private static final double TICKS_PER_INCH_EXT = MAX_EXT / MAX_DISTANCE;

    private static final int BUFFER = 10;

    /**
     * Constructor for Bot object
     * @param opMode
     */
    public Bot(LinearOpMode opMode) {
        this.opMode = opMode;
        init(opMode.hardwareMap);
    }

    /**
     * Initialize hardware mapping for robot
     * @param map
     */
    public void init(HardwareMap map){

        hwMap = map;

        //Connecting declared motors to classes of DcMotors and respected names
        leftMotorFront = hwMap.get(DcMotor.class, "left_front");
        leftMotorBack = hwMap.get(DcMotor.class, "left_back");
        rightMotorFront = hwMap.get(DcMotor.class, "right_front");
        rightMotorBack = hwMap.get(DcMotor.class, "right_back");


        leftMotorFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotorFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftMotorBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotorBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        leftLift = map.get(DcMotor.class, "left_lift");
        rightLift = map.get(DcMotor.class, "right_lift");

        extendArmMotor = map.get(DcMotor.class, "extend_arm");
        armPivotMotor = map.get(DcMotor.class, "pivot_arm");

        extendArmMotor = map.get(DcMotor.class, "extend_arm");
        armPivotMotor = map.get(DcMotor.class, "pivot_arm");


        leftPushoff = map.get(CRServo.class, "left_pushoff");
        rightPushoff = map.get(CRServo.class, "right_pushoff");

        //set encoders to 0 on init
        leftMotorFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotorBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotorFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotorBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        extendArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armPivotMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Set RunModes for Encoder Usage
        leftMotorFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftMotorBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotorFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotorBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        extendArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armPivotMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Set zero power behavior for motors
        leftMotorFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotorFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftMotorBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotorBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        extendArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armPivotMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Set Direction of each Motors
        // switch REVERSE and FORWARD if controls are opposite
        leftMotorFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftMotorBack.setDirection(DcMotorSimple.Direction.FORWARD);
        rightMotorFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightMotorBack.setDirection(DcMotorSimple.Direction.FORWARD);

        leftLift.setDirection(DcMotorSimple.Direction.FORWARD);
        rightLift.setDirection(DcMotorSimple.Direction.REVERSE);

        extendArmMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        armPivotMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        leftPushoff.setDirection(CRServo.Direction.FORWARD);
        rightPushoff.setDirection(CRServo.Direction.REVERSE);

        //Servos for intake on the map
        leftIntake = map.get(CRServo.class, "left_intake");
        rightIntake = map.get(CRServo.class, "right_intake");

    }

    /**
     * Set drive train power for mechanum drive
     * @param frontLeftPower front left motor power
     * @param backLeftPower back left motor power
     * @param frontRightPower front right motor power
     * @param backRightPower back right motor power
     */
    public void setDriveTrain(
           double frontLeftPower, double backLeftPower,
           double frontRightPower, double backRightPower
    ) {
       leftMotorFront.setPower(frontLeftPower);
       leftMotorBack.setPower(backLeftPower);
       rightMotorFront.setPower(frontRightPower);
       rightMotorBack.setPower(backRightPower);
    }

    /**
     * set Pushoff power for left and right
     */

    public void setPushoff(
            double pushoffPower
    ){
        leftPushoff.setPower(pushoffPower);
        rightPushoff.setPower(pushoffPower);
    }

    /**
     * Set Lift Power for hang
     * @param liftPower power for lift
     */
    public void setLift(
            double liftPower
    ){
        leftLift.setPower(liftPower);
        rightLift.setPower(liftPower);
    }

    /**
     * Set intake power
     * @param intakePower power for intake
     */
    public void setIntakePosition(
            double intakePower
    ) {
        leftIntake.setPower(intakePower);
        rightIntake.setPower(-intakePower);
    }

    /**
     * Run extension arm
     * @param power power of motor
     */
    public void setExtendPower(double power){ extendArmMotor.setPower(power);}

    public double getArmPosition(){ return armPivotMotor.getCurrentPosition();}

    /**
     * Run extension arm based on target position
     * @param targetPosition target position of pivot
     * @param power power of motor
     */
    public void autoPivotArm(
            int targetPosition, double power
    ) {
        armPivotMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armPivotMotor.setTargetPosition(targetPosition);
        armPivotMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armPivotMotor.setPower(power);
    }

    /**
     * Run Pivot Motor
     * @param power power of motor
     */
    public void setPivotPower(double power){ armPivotMotor.setPower(power);}

    /**
     * Get position of extension motor
     * @return encoder tick of extension motor
     */
    public double getExtendPos(){ return extendArmMotor.getCurrentPosition();}

    /**
     * Get position of pivot motor
     * @return encoder tick of pivot motor
     */
    public double getPivotArmPos(){ return armPivotMotor.getCurrentPosition();}

    /**
     * Get position of left lift motor
     * @return encoder tick of left lift motor
     */
    public double getLeftLiftPos() { return leftLift.getCurrentPosition();}

    /**
     * Get position of right lift motor
     * @return encoder tick of right lift motor
     */
    public double getRightLiftPos() { return rightLift.getCurrentPosition();}


    /**
     * Drive using encoders for auto
     * @param speed speed for power
     * @param distance distance to travel
     */
    public void encoderDrive(double speed, double distance) {
        int newfrontLeftTarget;
        int newfrontRightTarget;
        int newbackLeftTarget;
        int newbackRightTarget;

        //calculate distance to encoder ticks
        newfrontLeftTarget = leftMotorFront.getCurrentPosition() + (int) (-distance * COUNTS_PER_INCH);
        newfrontRightTarget = rightMotorFront.getCurrentPosition() + (int) (-distance * COUNTS_PER_INCH);
        newbackLeftTarget = leftMotorBack.getCurrentPosition() + (int) (-distance * COUNTS_PER_INCH);
        newbackRightTarget = rightMotorBack.getCurrentPosition() + (int) (-distance * COUNTS_PER_INCH);

        // set target position to hit for distance given
        leftMotorFront.setTargetPosition(newfrontLeftTarget);
        rightMotorFront.setTargetPosition(newfrontRightTarget);
        leftMotorBack.setTargetPosition(newbackLeftTarget);
        rightMotorBack.setTargetPosition(newbackRightTarget);

        // Turn On RUN_TO_POSITION
        leftMotorFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotorFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftMotorBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotorBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // start motion
        leftMotorFront.setPower(Math.abs(speed));
        rightMotorFront.setPower(Math.abs(speed));
        leftMotorBack.setPower(Math.abs(speed));
        rightMotorBack.setPower(Math.abs(speed));

        // Telemetry
        while (opMode.opModeIsActive() &&
                (leftMotorFront.isBusy() && rightMotorFront.isBusy() && leftMotorBack.isBusy() && rightMotorBack.isBusy())) {
            opMode.telemetry.addData("frontLeftEncoder", leftMotorFront.getCurrentPosition());
            opMode.telemetry.addData("frontRightEncoder", rightMotorFront.getCurrentPosition());
            opMode.telemetry.addData("backLeftEncoder", leftMotorBack.getCurrentPosition());
            opMode.telemetry.addData("backRightEncoder", rightMotorBack.getCurrentPosition());
            opMode.telemetry.update();
        }


        // Stop all motion;
        leftMotorFront.setPower(0);
        rightMotorFront.setPower(0);
        leftMotorBack.setPower(0);
        rightMotorBack.setPower(0);

        // Turn off RUN_TO_POSITION
        leftMotorFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotorFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftMotorBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotorBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // set encoders to 0 when function is completed
        // we do this to ensure the next set of functions are based at 0 instead of taking
        // the encoder value the function finishes at
        resetEncoder();
    }

    /**
     * Turn using encoders for auto (pivot)
     * @param speed speed for power
     * @param degrees angle degree to turn
     */
    public void encoderTurn(double speed, double degrees) {

        //calculate target count based on degrees to turn
        double turnCircumference = Math.PI * (degrees * 4 / 360) * (WHEEL_DIAMETER_INCHES * 2);
        int targetCounts = (int) (turnCircumference / DISTANCE_PER_ENCODER);

        // set target position of encoders based on degree to turn
        leftMotorFront.setTargetPosition(-targetCounts);
        rightMotorFront.setTargetPosition(targetCounts);
        leftMotorBack.setTargetPosition(-targetCounts);
        rightMotorBack.setTargetPosition(targetCounts);

        // Turn On RUN_TO_POSITION
        leftMotorFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotorFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftMotorBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotorBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // reset the timeout time and start motion.
        leftMotorFront.setPower(Math.abs(speed));
        rightMotorFront.setPower(Math.abs(speed));
        leftMotorBack.setPower(Math.abs(speed));
        rightMotorBack.setPower(Math.abs(speed));

        while (opMode.opModeIsActive() &&
                (leftMotorFront.isBusy() && leftMotorBack.isBusy() && rightMotorFront.isBusy() && rightMotorBack.isBusy())) {
            opMode.telemetry.addData("frontLeftEncoder", leftMotorFront.getCurrentPosition());
            opMode.telemetry.addData("frontRightEncoder", rightMotorFront.getCurrentPosition());
            opMode.telemetry.addData("backLeftEncoder", leftMotorBack.getCurrentPosition());
            opMode.telemetry.addData("backRightEncoder", rightMotorBack.getCurrentPosition());
            opMode.telemetry.update();
        }

        // Stop all motion;
        leftMotorFront.setPower(0);
        rightMotorFront.setPower(0);
        leftMotorBack.setPower(0);
        rightMotorBack.setPower(0);

        // Turn off RUN_TO_POSITION
        leftMotorFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotorFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftMotorBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotorBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // set encoders to 0 when function is completed
        // we do this to ensure the next set of functions are based at 0 instead of taking
        // the encoder value the function finishes at
        resetEncoder();
    }

    /**
     * Strafe using encoders for auto
     * NOTE: Pos distance is left strafe, Neg distance is right strafe
     * @param speed speed of motors
     * @param distance distance to travel
     */
    public void encoderStrafe(double speed, double distance) {
        int newfrontLeftTarget;
        int newfrontRightTarget;
        int newbackLeftTarget;
        int newbackRightTarget;

        // Determine new target position, and pass to motor controller
        newfrontLeftTarget = leftMotorFront.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH);
        newfrontRightTarget = rightMotorFront.getCurrentPosition() + (int) (-distance * COUNTS_PER_INCH);
        newbackLeftTarget = leftMotorBack.getCurrentPosition() + (int) (-distance * COUNTS_PER_INCH);
        newbackRightTarget = rightMotorBack.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH);

        //Set target position for motors
        leftMotorFront.setTargetPosition(newfrontLeftTarget);
        rightMotorFront.setTargetPosition(newfrontRightTarget);
        leftMotorBack.setTargetPosition(newbackLeftTarget);
        rightMotorBack.setTargetPosition(newbackRightTarget);

        // Turn On RUN_TO_POSITION
        leftMotorFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotorFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftMotorBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotorBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // reset the timeout time and start motion.
        leftMotorFront.setPower(Math.abs(speed));
        rightMotorFront.setPower(-Math.abs(speed));
        leftMotorBack.setPower(-Math.abs(speed));
        rightMotorBack.setPower(Math.abs(speed));

        //Telemetry
        while (opMode.opModeIsActive() &&
                (leftMotorFront.isBusy() && leftMotorBack.isBusy() && rightMotorFront.isBusy() && rightMotorBack.isBusy())) {
            opMode.telemetry.addData("frontLeftEncoder", leftMotorFront.getCurrentPosition());
            opMode.telemetry.addData("frontRightEncoder", rightMotorFront.getCurrentPosition());
            opMode.telemetry.addData("backLeftEncoder", leftMotorBack.getCurrentPosition());
            opMode.telemetry.addData("backRightEncoder", rightMotorBack.getCurrentPosition());
            opMode.telemetry.update();
            }

        // Stop all motion;
        leftMotorFront.setPower(0);
        rightMotorFront.setPower(0);
        leftMotorBack.setPower(0);
        rightMotorBack.setPower(0);

        // Turn off RUN_TO_POSITION
        leftMotorFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotorFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftMotorBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotorBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // set encoders to 0 when function is completed
        // we do this to ensure the next set of functions are based at 0 instead of taking
        // the encoder value the function finishes at
        resetEncoder();
    }

    /**
     * Sets encoder values of drive motors to 0
     */
    private void resetEncoder(){
        leftMotorBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotorFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotorBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotorFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }

    /**
     * Sequence for lifting bot for low hang
     */
    public void liftLow(){
        this.encoderLift(RIGHT_LIFT_MAX, LEFT_LIFT_MAX);
        this.setArmPos(1500);
        this.autoPush();
        this.encoderLift(RIGHT_LIFT_MAX/2,LEFT_LIFT_MAX/2);
        this.setArmPos(MAX_PIVOT-BUFFER);
        this.setExtendPos(5.0);
        this.encoderLift(RIGHT_LIFT_MIN/2, LEFT_LIFT_MIN/2);
        opMode.sleep(2000);
        //give time for bot to stop swayint
        this.encoderLift(RIGHT_LIFT_MAX- BUFFER*50, LEFT_LIFT_MAX-BUFFER*50);



    }

    /**
     * Sequence for lifting bot for high hang
     */
    //TODO FIX
    public void liftHigh(){

    }

    /**
     * Auto function to set pivot arm to position based on tick
     * @param tick position by encoder tick
     */
    public void setArmPos(int tick){
        armPivotMotor.setTargetPosition(tick);
        armPivotMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        armPivotMotor.setPower(0.75);

        while(opMode.opModeIsActive() && armPivotMotor.isBusy()){
            opMode.telemetry.addData("Pivot Pos: ", armPivotMotor.getCurrentPosition());
        }

        armPivotMotor.setPower(0);

        armPivotMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /**
     * Auto function to extend arm to a set position based on inches
     * NOTE: NOT TESTED
     * @param inches position in inches
     */
    public void setExtendPos(double inches){
        double target = inches * TICKS_PER_INCH_EXT;

        extendArmMotor.setTargetPosition((int) target);
        extendArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        extendArmMotor.setPower(1.0);

        while(opMode.opModeIsActive() && extendArmMotor.isBusy()){
            opMode.telemetry.addData("Extend Pos: ", extendArmMotor.getCurrentPosition());
        }

        extendArmMotor.setPower(0);

        extendArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /**
     * Auto function to extend arm to a set position based on tick
     * @param tick encoder tick position
     */
    public void setExtendPos(int tick){
        extendArmMotor.setTargetPosition(tick);
        extendArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        extendArmMotor.setPower(1.0);

        while(opMode.opModeIsActive() && extendArmMotor.isBusy()){
            opMode.telemetry.addData("Extend Pos: ", extendArmMotor.getCurrentPosition());
        }

        extendArmMotor.setPower(0);

        extendArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /**
     *
     * @param runTime how long the intake runs for
     * @param direction -1 for FORWARD: 1 for BACKWARD
     */
    public void runIntakeForTime(double runTime, int direction) {
        long startTime = System.currentTimeMillis();
        leftIntake.setPower(direction); // Full power for the intake
        rightIntake.setPower(-direction);
//goodluck
        // Run until the time is up
        while (opMode.opModeIsActive() && (System.currentTimeMillis() - startTime < runTime * 1000)) {
            opMode.telemetry.addData("Running intake", 1);
            opMode.telemetry.update();
        }

        // Stop the motor after the time has expired
        leftIntake.setPower(0);
        rightIntake.setPower(0);
    }

    /**
     * Auto for lift functionality
     * @param rPos right lift position
     * @param lPos left lift position
     */
    public void encoderLift(int rPos, int lPos){
        rightLift.setTargetPosition(rPos);
        leftLift.setTargetPosition(lPos);

        rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        rightLift.setPower(1.0);
        leftLift.setPower(1.0);

        while(opMode.opModeIsActive() && rightLift.isBusy() && leftLift.isBusy()){
            opMode.telemetry.addData("Left Lift Pos: ", leftLift.getCurrentPosition());
            opMode.telemetry.addData("Right Lift Pos: ", rightLift.getCurrentPosition());
            opMode.telemetry.update();
        }

        rightLift.setPower(0);
        leftLift.setPower(0);

        rightLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /**
     * Resets encoders for pivot and extend if it slips
     */
    public void d2EncoderReset(){
        armPivotMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extendArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armPivotMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        extendArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /**
     * Auto pushoff for lifting sequence
     */
    public void autoPush(){
        leftPushoff.setPower(-1.0);
        rightPushoff.setPower(-1.0);

        opMode.sleep(4500);

        leftPushoff.setPower(0);
        rightPushoff.setPower(0);
    }

    /**
     * Auto Intake Function
     * @param time how long to run intake
     */
    public void autoIntake(double time){
        this.setArmPos(-230);
        this.setExtendPos(8.39);
        this.runIntakeForTime(time, 1);
        this.setExtendPos(0.0);
        this.setArmPos(0);
    }

    public void autoIntake(){
        this.setArmPos(-230);
        this.setExtendPos(8.39);
        this.setIntakePosition(1);
    }

    /**
     * Used for bypass on extend if encoder not zeroed correctly
     * @param power power to motor
     */
    public void runExtend(double power){ extendArmMotor.setPower(power); }

}
