package org.firstinspires.ftc.team8923_CENTERSTAGE;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.bosch.BNO055IMU;


abstract public class BaseOpMode extends LinearOpMode {
    DcMotor motorFL;
    DcMotor motorFR;
    DcMotor motorBL;
    DcMotor motorBR;
    DcMotor motorIntakeWheels;

    // constants
    static final double TICKS_PER_REVOLUTION = 537.6; // Neverest orbital 20, 7 pulse per revolution, 4 * 7 * 19.2 (the calculations)
    static final double GEAR_RATIO = 1.0;
    static final double WHEEL_DIAMETER = 4.0; // inches
    static final double TICKS_PER_INCH =  (TICKS_PER_REVOLUTION * GEAR_RATIO) / (WHEEL_DIAMETER * Math.PI);
    static final double INCHES_PER_REVOLUTION = Math.PI * WHEEL_DIAMETER;
    static final double INCHES_PER_TICK = INCHES_PER_REVOLUTION / TICKS_PER_REVOLUTION;
    static final double INTAKE_SPEED = 0.8;

    //imu constants
    public IMU imu;
    public double startAngle;
    public static final double TURNING_KP = 0.008;
    public static final int ROBOT_HEADING_TOLERANCE_DEGREES = 1;
    public static final double MAXIMUM_TURN_POWER_AUTONOMOUS = 0.7;
    public static final double MINIMUM_TURN_POWER = 0.05;

    public double referenceAngle;

    public void initHardware() {
        // drive motors
        motorFL = hardwareMap.dcMotor.get("motorFL");
        motorFR = hardwareMap.dcMotor.get("motorFR");
        motorBL = hardwareMap.dcMotor.get("motorBL");
        motorBR = hardwareMap.dcMotor.get("motorBR");

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



        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
       /* BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";*/

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));
        imu.initialize(parameters);

        sleep(2000);// a wait, so the imu can process

    }

    // mecanum drive method
    public void driveMecanum(double x, double y, double turning) {
        motorFL.setPower(x + y + turning);
        motorFR.setPower(y - x - turning);
        motorBL.setPower(y - x + turning);
        motorBR.setPower(x + y - turning);
    }

    public void driveMecanumGyro(double driveAngle, double drivePower, double turnPower) {
        // proportionality constant to correct angle error
        double kpConstant = 0.01;

        // Calculate x and y components of drive power, where y is forward (0 degrees) and x is right (-90 degrees)
        double x = drivePower * Math.cos(Math.toRadians(driveAngle));
        double y = drivePower * Math.sin(Math.toRadians(driveAngle));

        double robotAngle;

        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();

        robotAngle = orientation.getYaw(AngleUnit.DEGREES);

        if (turnPower != 0) {
            referenceAngle = robotAngle;
        }

        double angleError;
        if (gamepad1.left_bumper) {
            angleError = robotAngle - referenceAngle;
        } else {
            angleError = 0;
            referenceAngle = robotAngle;
        }

        if (angleError > 180.0) {
            angleError -= 360.0;
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

    double calculateDistance(double deltaX, double deltaY) {
        return Math.sqrt(Math.pow(deltaX, 2) + Math.pow(deltaY, 2));
    }

}
