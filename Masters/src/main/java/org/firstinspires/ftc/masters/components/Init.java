package org.firstinspires.ftc.masters.components;

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

    private final DcMotor intake, intakeExtendo;
    private final Servo intakeLeft, intakeRight;

    private final DcMotor outtakeSlideLeft, outtakeSlideRight;

    private final Servo led, claw;
    private final Servo wrist, angleLeft, angleRight, position;
//    private final Servo ptoRight, ptoLeft, hangLeft, hangRight;
    private IMU imu;

    public Telemetry telemetry;

    public Init(HardwareMap hardwareMap) {
        // Read from the hardware maps
        leftFrontMotor = hardwareMap.get(DcMotorEx.class, "frontLeft");
        rightFrontMotor = hardwareMap.get(DcMotorEx.class, "frontRight");
        leftRearMotor = hardwareMap.get(DcMotorEx.class, "backLeft");
        rightRearMotor = hardwareMap.get(DcMotorEx.class, "backRight");

        // Set the drive motor direction:
        leftFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        rightFrontMotor.setDirection(DcMotor.Direction.FORWARD);
        leftRearMotor.setDirection(DcMotor.Direction.REVERSE);
        rightRearMotor.setDirection(DcMotor.Direction.FORWARD);

        // Don't use the encoders for motor odometry
        leftFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRearMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRearMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Engage the brakes when the robot cuts off power to the motors
        leftFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRearMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRearMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

//        imu = hardwareMap.get(IMU.class, "imu");
//        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
//                org.firstinspires.ftc.masters.drive.DriveConstants.LOGO_FACING_DIR, org.firstinspires.ftc.masters.drive.DriveConstants.USB_FACING_DIR));
//        imu.initialize(parameters);

        // Initialize intake motors and servos
        claw = hardwareMap.servo.get("claw");
        position = hardwareMap.servo.get("position");
        wrist = hardwareMap.servo.get("wrist");
        angleLeft = hardwareMap.servo.get("angleLeft");
        angleRight = hardwareMap.servo.get("angleRight");

        intake = hardwareMap.dcMotor.get("intake");
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeExtendo = hardwareMap.dcMotor.get("intakeExtendo");
        intakeExtendo.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeExtendo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeExtendo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        intakeLeft = hardwareMap.servo.get("intakeLeft");
        intakeRight = hardwareMap.servo.get("intakeRight");

        outtakeSlideRight = hardwareMap.dcMotor.get("vertSlideRight");


        outtakeSlideLeft = hardwareMap.dcMotor.get("vertSlideLeft");
        outtakeSlideLeft.setDirection(DcMotorSimple.Direction.REVERSE);

//        ptoRight = hardwareMap.servo.get("ptoRight");
//        ptoLeft = hardwareMap.servo.get("ptoLeft");
//        hangLeft = hardwareMap.servo.get("hangLeft");
//        hangRight = hardwareMap.servo.get("hangRight");

        led = hardwareMap.servo.get("led");

    }

    public DcMotorEx getLeftFrontMotor(){return leftFrontMotor;}
    public DcMotorEx getRightFrontMotor(){return rightFrontMotor;}
    public DcMotorEx getLeftRearMotor(){return leftRearMotor;}
    public DcMotorEx getRightRearMotor(){return rightRearMotor;}

    public DcMotor getIntake() {
        return intake;
    }

    public DcMotor getIntakeExtendo() {
        return intakeExtendo;
    }

    public Servo getIntakeLeft() {
        return intakeLeft;
    }

    public Servo getIntakeRight() {
        return intakeRight;
    }

    public DcMotor getOuttakeSlideLeft() {
        return outtakeSlideLeft;
    }

    public DcMotor getOuttakeSlideRight() {
        return outtakeSlideRight;
    }

    public Servo getLed() {
        return led;
    }

    public Servo getClaw() {
        return claw;
    }

    public Servo getWrist() {
        return wrist;
    }

    public Servo getAngleLeft() {
        return angleLeft;
    }

    public Servo getAngleRight() {
        return angleRight;
    }

    public Servo getPosition() {
        return position;
    }

//    public Servo getPtoRight() {return ptoRight;}
//
//    public Servo getPtoLeft(){ return  ptoLeft;}
//     public Servo getHangLeft() {return hangLeft;}
//    public Servo getHangRight() {return hangRight;}

    public Telemetry getTelemetry() {
        return telemetry;
    }

}