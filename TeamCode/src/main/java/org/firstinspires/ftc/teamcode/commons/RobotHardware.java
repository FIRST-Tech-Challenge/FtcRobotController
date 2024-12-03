package org.firstinspires.ftc.teamcode.commons;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class RobotHardware {

    // Declare motors and servos
    public DcMotor frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor;
    public DcMotor leftClaw, rightClaw, leftHang, rightHang;
    public Servo leftRotation, rightRotation, rightClawServo, leftClawServo;

    // Initialization method to set up all hardware
    public void init(HardwareMap hardwareMap) {
        // Map each motor to its hardware configuration name
        frontLeftMotor = hardwareMap.get(DcMotor.class, "leftFront");
        backLeftMotor = hardwareMap.get(DcMotor.class, "leftBack");
        frontRightMotor = hardwareMap.get(DcMotor.class, "rightFront");
        backRightMotor = hardwareMap.get(DcMotor.class, "rightBack");

        leftClaw = hardwareMap.get(DcMotor.class, "leftClaw");
        rightClaw = hardwareMap.get(DcMotor.class, "rightClaw");

        leftHang = hardwareMap.get(DcMotor.class, "leftHang");
        rightHang = hardwareMap.get(DcMotor.class, "rightHang");

        leftRotation = hardwareMap.get(Servo.class, "leftRotation");
        rightRotation = hardwareMap.get(Servo.class, "rightRotation");
        rightClawServo = hardwareMap.get(Servo.class, "rightClawServo");
        leftClawServo = hardwareMap.get(Servo.class, "leftClawServo");

        // Configure motor directions if needed
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        // TODO: is it ok to set backleft in reverse or should it be back right
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftClaw.setDirection(DcMotor.Direction.REVERSE);

        // Set zero power behavior for motors
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftHang.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightHang.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
}
