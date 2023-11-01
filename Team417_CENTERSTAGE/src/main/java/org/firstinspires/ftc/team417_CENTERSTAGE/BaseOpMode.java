package org.firstinspires.ftc.team417_CENTERSTAGE;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.openftc.easyopencv.OpenCvCamera;

import drive.SampleMecanumDrive;

abstract class BaseOpMode extends LinearOpMode {
    OpenCvCamera camera;

    SampleMecanumDrive drive;

    //Declares drive-motors
    public DcMotor FR;
    public DcMotor FL;
    public DcMotor BR;
    public DcMotor BL;

    public DcMotor intakeMotor;
    public DcMotor armMotor;
    public Servo dumperServo;
    public Servo gateServo;

    static final double TICKS_PER_REVOLUTION = 5281.1; // 5203 Series Yellow Jacket Motor
    static final double GEAR_RATIO = 1.0;
    static final double WHEEL_DIAMETER = 3.7; // inches
    static final double TICKS_PER_INCH = (TICKS_PER_REVOLUTION * GEAR_RATIO) / (WHEEL_DIAMETER * Math.PI);
    static final double INCHES_PER_REVOLUTION = Math.PI * WHEEL_DIAMETER;
    static final double INCHES_PER_TICK = INCHES_PER_REVOLUTION / TICKS_PER_REVOLUTION;

    //Declares IMU
    //public BNO055IMU imu;

    //Initializes motors, servos, and sensors
    public void initializeHardware() {
        //Drive Motors
        FL = initializeMotor("FLMotor", DcMotor.Direction.FORWARD);
        FR = initializeMotor("FRMotor", DcMotor.Direction.REVERSE);
        BL = initializeMotor("BLMotor", DcMotor.Direction.FORWARD);
        BR = initializeMotor("BRMotor", DcMotor.Direction.REVERSE);

        //Mechanism Motors
        intakeMotor = initializeMotor("IntakeMotor");

        intakeMotor = hardwareMap.get(DcMotor.class, "IntakeMotor");
        armMotor = hardwareMap.get(DcMotor.class, "ArmMotor");

        intakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intakeMotor.setDirection(DcMotor.Direction.FORWARD);

        dumperServo = hardwareMap.get(Servo.class, "DumperServo");
        gateServo = hardwareMap.get(Servo.class, "GateServo");

        /*
        // Sets up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample op-mode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";

        // Retrieves and initializes the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        */

        // Waits so the imu can process
        sleep(2000);
    }

    public DcMotor initializeMotor(String motorName, DcMotor.Direction direction) {
        DcMotor motor = hardwareMap.get(DcMotor.class, motorName);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setDirection(direction);
        return motor;
    }

    public void mecanumDrive(double x, double y, double rot) {
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rot), 1);

        double frontLeftPower = (y + x + rot) / denominator;
        double frontRightPower = (y - x - rot) / denominator;
        double backLeftPower = (y - x + rot) / denominator;
        double backRightPower = (y + x - rot) / denominator;

        FL.setPower(frontLeftPower);
        FR.setPower(frontRightPower);
        BL.setPower(backLeftPower);
        BR.setPower(backRightPower);
    }

    public void runIntakeMechanism(double speed) {
        intakeMotor.setPower(speed);
    }

    public void moveOutputMechanism(double position) {
        dumperServo.setPosition(position);
    }
}
