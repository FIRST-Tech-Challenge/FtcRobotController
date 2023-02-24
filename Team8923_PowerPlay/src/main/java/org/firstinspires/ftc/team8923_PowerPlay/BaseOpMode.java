package org.firstinspires.ftc.team8923_PowerPlay;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

abstract public class BaseOpMode extends LinearOpMode {

    // Declared drive motors
    DcMotor motorFL;
    DcMotor motorFR;
    DcMotor motorBL;
    DcMotor motorBR;
    double driveSpeed = 1.0;

    // declare linear slide motors and servo
    DcMotor motorSlideLeft;
    DcMotor motorSlideRight;
    Servo servoClaw;

    public double referenceAngle;

    public BNO055IMU imu;
    double bottomMotorSlideLeft;
    double bottomMotorSlideRight;

    public void initHardware() {
        // init drive motors
        motorFL = hardwareMap.dcMotor.get("motorFL");
        motorFR = hardwareMap.dcMotor.get("motorFR");
        motorBL = hardwareMap.dcMotor.get("motorBL");
        motorBR = hardwareMap.dcMotor.get("motorBR");

        // reset encoder
        motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorFL.setDirection(DcMotor.Direction.FORWARD);
        motorFR.setDirection(DcMotor.Direction.REVERSE);
        motorBL.setDirection(DcMotor.Direction.FORWARD);
        motorBR.setDirection(DcMotor.Direction.REVERSE);

        // init linear slide motors and claw servo
        motorSlideLeft = hardwareMap.dcMotor.get("motorSlideLeft");
        motorSlideRight = hardwareMap.dcMotor.get("motorSlideRight");
        servoClaw = hardwareMap.servo.get("servoClaw");

       // reset encoder
        motorSlideLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorSlideRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorSlideLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorSlideRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorSlideLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorSlideRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorSlideLeft.setDirection(DcMotor.Direction.FORWARD);
        motorSlideRight.setDirection(DcMotor.Direction.REVERSE);

        bottomMotorSlideLeft = motorSlideLeft.getCurrentPosition();
        bottomMotorSlideRight = motorSlideRight.getCurrentPosition();

        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        sleep(2000);// a wait, so the imu can process
    }

    public void driveMecanum(double driveAngle, double drivePower, double turnPower) {
        // Calculate x and y components of drive power, where y is forward (0 degrees) and x is right (-90 degrees)
        double x = drivePower * Math.cos(Math.toRadians(driveAngle));
        double y = drivePower * Math.sin(Math.toRadians(driveAngle));

        double powerFL = x + y + turnPower;
        double powerFR = y - x - turnPower;
        double powerBL = y - x + turnPower;
        double powerBR = y + x - turnPower;

        // gets the largest power
        double scaleFactor = Math.max(Math.max(powerFL, powerFR), Math.max(powerBL, powerBR));
        // scale the power between the range of -1 and 1
        /*if (scaleFactor > driveSpeed) {
            powerFL /= scaleFactor;
            powerFR /= scaleFactor;
            powerBL /= scaleFactor;
            powerBR /= scaleFactor;
        }*/

        powerFL *= driveSpeed;
        powerFR *= driveSpeed;
        powerBL *= driveSpeed;
        powerBR *= driveSpeed;

        motorFL.setPower(powerFL);
        motorFR.setPower(powerFR);
        motorBL.setPower(powerBL);
        motorBR.setPower(powerBR);
    }

    public void driveMecanumGyro(double driveAngle, double drivePower, double turnPower){
        // proportionality constant to correct angle error
        double kpConstant = 0.01;

        // Calculate x and y components of drive power, where y is forward (0 degrees) and x is right (-90 degrees)
        double x = drivePower * Math.cos(Math.toRadians(driveAngle));
        double y = drivePower * Math.sin(Math.toRadians(driveAngle));

        double robotAngle = imu.getAngularOrientation().firstAngle;

        double angleError = robotAngle - referenceAngle;

        if (angleError > 180.0) {
            angleError -= 360.0;
        } else if(angleError < -180.0) {

        }
        double pivotCorrection = angleError * kpConstant;

        double sumPivotTurning = pivotCorrection + turnPower;

        double powerFL = x + y + sumPivotTurning;
        double powerFR = y - x - sumPivotTurning;
        double powerBL = y - x + sumPivotTurning;
        double powerBR = y + x - sumPivotTurning;

        motorFL.setPower(powerFL);
        motorFR.setPower(powerFR);
        motorBL.setPower(powerBL);
        motorBR.setPower(powerBR);
    }

    // Used for calculating distances between 2 points
    double calculateDistance(double deltaX, double deltaY)
    {
        return Math.sqrt(Math.pow(deltaX, 2) + Math.pow(deltaY, 2));
    }
}
