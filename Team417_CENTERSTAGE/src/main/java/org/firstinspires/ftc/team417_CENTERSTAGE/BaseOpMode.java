package org.firstinspires.ftc.team417_CENTERSTAGE;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import drive.DriveConstants;
import drive.SampleMecanumDrive;

public abstract class BaseOpMode extends LinearOpMode {
    SampleMecanumDrive drive;

    //Declares drive-motors
    public DcMotor FR = null; // null because of DevBot
    public DcMotor FL = null;
    public DcMotor BR = null;
    public DcMotor BL = null;

    public DcMotor intakeMotor = null;
    public DcMotor armMotor = null;
    final public int ARM_MOTOR_MIN_POSITION = 0;
    final public int ARM_MOTOR_MAX_POSITION = 4200;
    public Servo dumperServo = null;
    public Servo gateServo = null;

    static final double TICKS_PER_REVOLUTION = 537.7 * (24.0/27); // 5203 Series Yellow Jacket Motor, robot was overshooting so
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
        if(DriveConstants.isDevBot) {
            FL = initializeMotor("leftFront", DcMotor.Direction.REVERSE);
            FR = initializeMotor("rightFront", DcMotor.Direction.FORWARD);
            BL = initializeMotor("leftBack", DcMotor.Direction.REVERSE);
            BR = initializeMotor("rightBack", DcMotor.Direction.FORWARD);
        } else {
            FL = initializeMotor("FLMotor", DcMotor.Direction.REVERSE);
            FR = initializeMotor("FRMotor", DcMotor.Direction.FORWARD);
            BL = initializeMotor("BLMotor", DcMotor.Direction.REVERSE);
            BR = initializeMotor("BRMotor", DcMotor.Direction.FORWARD);

            //Mechanism Motors
            intakeMotor = initializeMotor("IntakeMotor", DcMotor.Direction.FORWARD);
            armMotor = initializeMotor("ArmMotor", DcMotor.Direction.FORWARD);//DcMotor.RunMode.RUN_TO_POSITION);

            //Mechanism Servos
            dumperServo = initializeServo("DumperServo", Servo.Direction.FORWARD);
            gateServo = initializeServo("GateServo", Servo.Direction.FORWARD);
        }
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

    public DcMotor initializeMotor(String motorName, DcMotorSimple.Direction direction, DcMotor.RunMode mode) {
        DcMotor motor = hardwareMap.get(DcMotor.class, motorName);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(mode);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setDirection(direction);
        return motor;
    }

    public DcMotor initializeMotor(String motorName, DcMotor.Direction direction) {
        DcMotor motor = hardwareMap.get(DcMotor.class, motorName);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setDirection(direction);
        return motor;
    }

    public Servo initializeServo(String servoName, Servo.Direction direction) {
        Servo servo = hardwareMap.get(Servo.class, servoName);
        servo.setDirection(direction);
        return servo;
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

    public void tiltDumper() {
        dumperServo.setPosition(0.32);
    }

    public void resetDumper() {
        dumperServo.setPosition(0.48);
    }

    public void dumpDumper() {
        dumperServo.setPosition(0);
    }

    enum DumperAction {
        DUMPING,
        RESETTING,
        STOPPING
    }
    public void moveDumper(DumperAction dumperAction) {
        switch (dumperAction) {
            case DUMPING:
                dumperServo.setPosition(1);
            case RESETTING:
                dumperServo.setPosition(0);
            case STOPPING:
                dumperServo.setPosition(dumperServo.getPosition());
            default:
        }
    }

    public void openGate() {
        gateServo.setPosition(0.5);
    }

    public void closeGate() {
        gateServo.setPosition(1);
    }

    public int[] armPositions = new int[] {ARM_MOTOR_MIN_POSITION, ARM_MOTOR_MIN_POSITION + ((ARM_MOTOR_MAX_POSITION - ARM_MOTOR_MIN_POSITION) / 2), ARM_MOTOR_MAX_POSITION};

    public void positionArm(int armPositionIndex) {
        if (armPositionIndex >= 0 && armPositionIndex <= armPositions.length - 1) {
            armMotor.setTargetPosition(armPositions[armPositionIndex]);
        }
    }

    public void moveArm(double speed) {
        if (armMotor.getCurrentPosition() > ARM_MOTOR_MAX_POSITION) {
            speed = -0.1;
        } else if (armMotor.getCurrentPosition() < ARM_MOTOR_MIN_POSITION) {
            speed = 0.1;
        }
        armMotor.setPower(speed);
    }

    public double epsilon = 0.0001;
    public boolean isEpsilonEquals(double a, double b) {
        return (Math.abs(a) + epsilon >= Math.abs(b) && Math.abs(a) - epsilon <= Math.abs(b));
    }
}
