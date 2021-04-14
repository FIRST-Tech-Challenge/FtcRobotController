package org.firstinspires.ftc.team417_2020;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.team417_2020.Resources.FIRFilter;
import org.firstinspires.ftc.team417_2020.Resources.PIDFilter;
import org.firstinspires.ftc.team417_2020.Resources.Polynomial;


abstract public class MasterOpMode extends LinearOpMode {

    public Robot robot = new Robot(this);
    // Declare drive motors
    public DcMotor motorFL = null; // hub 2 port 0
    public DcMotor motorFR = null; // hub 1 port 0
    public DcMotor motorBL = null; // hub 2 port 1
    public DcMotor motorBR = null; // hub 1 port 1
    public DcMotor motorWobbleGoalArm = null; //
    public Servo wobbleGoalGrabber = null;//

    public BNO055IMU imu;
    //initial position of servo
    static final double WOBBLE_GOAL_GRABBER_OUT = 0.0;
    static final double COUNTS_PER_MOTOR_REV = 537.6; // 40:1 motor    1120
    static final double DRIVE_GEAR_REDUCTION = 1.0; // This is < 1.0 if geared UP    16.0 / 24.0
    static final double WHEEL_DIAMETER_INCHES = 4.0; // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * Math.PI);
    static final double WOBBLE_GOAL_GRABBER_IN = 0.5;
    static final int WOBBLE_GOAL_ARM_UP = 500;
    static final int WOBBLE_GOAL_ARM_DOWN = 150;
    static final int WOBBLE_GOAL_ARM_MID = 300;


    PIDFilter turnFilter;
    PIDFilter moveFilter;
    FIRFilter accelerationFilter;
    int filterLength = 10;

    /**
     * Initialize all hardware, including motors, servos, and IMU
     */
    protected void initializeHardware()
    {
        turnFilter = new PIDFilter(0.015, 0, 0.00);
        moveFilter = new PIDFilter(0.04, 0, 0);
        // weights for weighted average
        double[] filterCoefficients = {1};
        accelerationFilter = new FIRFilter(new Polynomial(filterCoefficients),filterLength);
        accelerationFilter.values = new double[filterLength];


        // Initialize motors to be the hardware motors
        motorFL = hardwareMap.dcMotor.get("motorFL");
        motorFR = hardwareMap.dcMotor.get("motorFR");
        motorBL = hardwareMap.dcMotor.get("motorBL");
        motorBR = hardwareMap.dcMotor.get("motorBR");
        motorWobbleGoalArm = hardwareMap.dcMotor.get("motorWobbleGoalArm");
        wobbleGoalGrabber = hardwareMap.servo.get("wobbleGoalGrabber");

        motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorWobbleGoalArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorWobbleGoalArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorWobbleGoalArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // reverse front and back right motors
        motorFL.setDirection(DcMotor.Direction.FORWARD);
        motorBL.setDirection(DcMotor.Direction.FORWARD);
        motorFR.setDirection(DcMotor.Direction.REVERSE);
        motorBR.setDirection(DcMotor.Direction.REVERSE);

        // set motor power to 0
        motorFL.setPower(0);
        motorFR.setPower(0);
        motorBL.setPower(0);
        motorBR.setPower(0);
        motorWobbleGoalArm.setPower(0);

        wobbleGoalGrabber.setPosition(WOBBLE_GOAL_GRABBER_IN);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "Adaf'" +
                "ruitIMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        telemetry.log().setCapacity(8);


    } //-----------------------END OF INITIALIZATION SOFTWARE------------------------

    /**
     * returns the coterminal angle between -180 to 180
     * @param angle
     * Double angle to normalize
     */
    public double adjustAngles(double angle)
    {
        while(angle > 180)
            angle -= 360;
        while(angle < -180)
            angle += 360;
        return angle;
    }



    /**
     * Set drive motor powers for a mecanum drive robot
     * @param angle
     * Without turning the robot, the angle that the robot drives at
     * @param drivePower
     * The speed at which to drive the robot
     * @param rotationalPower
     * The speed at which to turn the robot
     */
    public void mecanumDrive(double angle, double drivePower, double rotationalPower) {

        double x = drivePower * Math.cos(angle);
        double y = drivePower * Math.sin(angle);


        double frontLeft = y + x + rotationalPower;
        double frontRight = y - x - rotationalPower;
        double backLeft = y - x + rotationalPower;
        double backRight = y + x - rotationalPower;

        // get the largest power
        double powerScalar = returnLargestValue(frontLeft, frontRight, backLeft, backRight);

        // scale the power to keep the wheels proportional and between the range of -1 and 1
        if (powerScalar > 1) {
            frontLeft /= powerScalar;
            frontRight /= powerScalar;
            backLeft /= powerScalar;
            backRight /= powerScalar;
        }

        motorFL.setPower(frontLeft);
        motorFR.setPower(frontRight);
        motorBL.setPower(backLeft);
        motorBR.setPower(backRight);
    }

    /**
     * Returns the largest value from any number of doubles
     * @param numberArray
     * the doubles to evaluate
     */
    public double returnLargestValue(double ... numberArray) {
        double max = Double.MIN_VALUE;
        for ( double num : numberArray ) {
            if (num > max) {
                max = num;
            }
        }
        return max;
    }

    /**
     * Runs DcMotor at any power using encoder
     * @param motor
     * @param targetPosition
     * @param power
     */
    public void runMotorToPosition(DcMotor motor, int targetPosition, double power) {

        motor.setTargetPosition(targetPosition);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(power);


    }



}
