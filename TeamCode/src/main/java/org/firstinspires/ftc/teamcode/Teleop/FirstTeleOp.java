package org.firstinspires.ftc.teamcode.Teleop;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

public class FirstTeleOp extends LinearOpMode {
    DcMotorEx lb, lf, rb, rf, intake1, intake2, vertical;
    Servo inClaw, depClaw, inWrist, depWrist, transfer;
    ColorSensor transColor, inColor;
    IMU imu;

    @Override
    public void runOpMode() throws InterruptedException {
        lb       = hardwareMap.get(DcMotorEx .class, "lb");
        lf       = hardwareMap.get(DcMotorEx.class, "lf");
        rb       = hardwareMap.get(DcMotorEx.class, "rb");
        rf       = hardwareMap.get(DcMotorEx.class, "rf");
        intake1  = hardwareMap.get(DcMotorEx.class, "intake1");
        intake2  = hardwareMap.get(DcMotorEx.class, "intake2");
        vertical = hardwareMap.get(DcMotorEx.class, "vertical");
        inClaw   = hardwareMap.get(Servo.class, "inClaw");
        depClaw  = hardwareMap.get(Servo.class, "depClaw");
        inWrist  = hardwareMap.get(Servo.class, "inWrist");
        depWrist = hardwareMap.get(Servo.class, "depWrist");
        transfer = hardwareMap.get(Servo.class, "transfer");
        imu      = hardwareMap.get(IMU.class, "imu");



        waitForStart();

        while (opModeIsActive()) {

        }

    }
}
