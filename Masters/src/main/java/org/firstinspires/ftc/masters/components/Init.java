package org.firstinspires.ftc.masters.components;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Init {

    private final DcMotor leftFrontMotor;
    private final DcMotor rightFrontMotor;
    private final DcMotor leftRearMotor;
    private final DcMotor rightRearMotor;

    private final Servo elbow1, elbow2, fingers;
    private final DcMotor extension1;
    private final DcMotor extension2;

    private final Servo slideServo1, slideServo2, stringServo;
    private final DcMotor wheelMotor;
    BNO055IMU imu;

    public Telemetry telemetry;

    public Init(HardwareMap hardwareMap) {
        // Read from the hardware maps
        leftFrontMotor = hardwareMap.dcMotor.get("frontLeft");
        rightFrontMotor = hardwareMap.dcMotor.get("frontRight");
        leftRearMotor = hardwareMap.dcMotor.get("backLeft");
        rightRearMotor = hardwareMap.dcMotor.get("backRight");

        // Reset the encoder values
        leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRearMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRearMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set the drive motor direction:
        leftFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        rightFrontMotor.setDirection(DcMotor.Direction.FORWARD);
        leftRearMotor.setDirection(DcMotor.Direction.REVERSE);
        rightRearMotor.setDirection(DcMotor.Direction.FORWARD);

        // Don't use the encoders for motor odometry
        leftFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRearMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRearMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Engage the brakes when the robot cuts off power to the motors
        leftFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRearMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRearMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = null;

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        // Initialize intake motors and servos
        wheelMotor = hardwareMap.dcMotor.get("wheelMotor");
        slideServo1 = hardwareMap.servo.get("slideServo1");
        slideServo2 = hardwareMap.servo.get("slideServo2");
        stringServo = hardwareMap.servo.get("stringServo");

        // Initialize outtake motors and servos
        extension1 = hardwareMap.dcMotor.get("extension1");
        extension2 = hardwareMap.dcMotor.get("extension2");

        extension2.setDirection(DcMotor.Direction.REVERSE);

        extension1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extension2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

       elbow1 = hardwareMap.servo.get("elbow1");
       elbow2 = hardwareMap.servo.get("elbow2");
       fingers = hardwareMap.servo.get("fingers");

    }

    public DcMotor getLeftFrontMotor(){return leftFrontMotor;}
    public DcMotor getRightFrontMotor(){return rightFrontMotor;}
    public DcMotor getLeftRearMotor(){return leftRearMotor;}
    public DcMotor getRightRearMotor(){return rightRearMotor;}

    public DcMotor getWheelMotor(){return wheelMotor;}
    public Servo getSlideServo1(){return slideServo1;}
    public Servo getSlideServo2(){return slideServo2;}
    public Servo getStringServo(){return stringServo;}

    public DcMotor getExtension1(){return extension1;}
    public DcMotor getExtension2(){return extension2;}

    public Servo getElbow1(){return elbow1;}
    public Servo getElbow2(){return elbow2;}
    public Servo getFingers(){return fingers;}
}