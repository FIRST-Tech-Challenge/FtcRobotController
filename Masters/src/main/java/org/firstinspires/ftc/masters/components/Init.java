package org.firstinspires.ftc.masters.components;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Init {

    private final DcMotorEx leftFrontMotor;
    private final DcMotorEx rightFrontMotor;
    private final DcMotorEx leftRearMotor;
    private final DcMotorEx rightRearMotor;

    private final Servo fingers;
    private final Servo elbow1, elbow2;
    private final DcMotor extension1;
    private final DcMotor extension2;

    private final Servo slideServo1, slideServo2, stringServo;
    private final DcMotor wheelMotor;
    IMU imu;

    public Telemetry telemetry;

    public Init(HardwareMap hardwareMap) {
        // Read from the hardware maps
        leftFrontMotor = hardwareMap.get(DcMotorEx.class, "frontLeft");
        rightFrontMotor = hardwareMap.get(DcMotorEx.class, "frontRight");
        leftRearMotor = hardwareMap.get(DcMotorEx.class, "backLeft");
        rightRearMotor = hardwareMap.get(DcMotorEx.class, "backRight");

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

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                org.firstinspires.ftc.masters.drive.DriveConstants.LOGO_FACING_DIR, org.firstinspires.ftc.masters.drive.DriveConstants.USB_FACING_DIR));
        imu.initialize(parameters);

        // Initialize intake motors and servos
        wheelMotor = hardwareMap.dcMotor.get("wheelMotor");
        slideServo1 = hardwareMap.servo.get("slideServo1");
        slideServo2 = hardwareMap.servo.get("slideServo2");
        stringServo = hardwareMap.servo.get("stringServo");

        // Initialize outtake motors and servos
        extension1 = hardwareMap.dcMotor.get("extension1");
        extension2 = hardwareMap.dcMotor.get("extension2");

        extension2.setDirection(DcMotorSimple.Direction.REVERSE);
        extension2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extension2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

       elbow1 = hardwareMap.servo.get("elbow1");
       elbow2 = hardwareMap.servo.get("elbow2");
       fingers = hardwareMap.servo.get("fingers");

    }

    public DcMotorEx getLeftFrontMotor(){return leftFrontMotor;}
    public DcMotorEx getRightFrontMotor(){return rightFrontMotor;}
    public DcMotorEx getLeftRearMotor(){return leftRearMotor;}
    public DcMotorEx getRightRearMotor(){return rightRearMotor;}

    public DcMotor getWheelMotor(){return wheelMotor;}
    public Servo getSlideServo1(){return slideServo1;}
    public Servo getSlideServo2(){return slideServo2;}
    public Servo getStringServo(){return stringServo;}

    public DcMotor getExtension1(){return extension1;}
    public DcMotor getExtension2(){return extension2;}

    public Servo getElbow1(){return elbow1;}
    public Servo getElbow2(){return elbow2;}
    public Servo getFingers(){return fingers;}

    public IMU getImu(){return imu;}

}