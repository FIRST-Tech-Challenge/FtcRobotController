package org.firstinspires.ftc.Team19567;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import java.lang.Math;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Basic: Iterative OpMode", group="Iterative Opmode")
public class TestOP extends OpMode
{
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDCFront = null;
    private DcMotor rightDCFront = null;
    private DcMotor leftDCBack = null;
    private DcMotor rightDCBack = null;
    private boolean slowMode = false;
    private double acc = 1.0;
    private Servo servo = null;
    private boolean isServoMaxed = false;

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        leftDCFront  = hardwareMap.get(DcMotor.class, "leftFront");
        rightDCFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftDCBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightDCBack = hardwareMap.get(DcMotor.class, "rightBack");
        servo = hardwareMap.get(Servo.class, "servo1");

        leftDCFront.setDirection(DcMotor.Direction.FORWARD);
        rightDCFront.setDirection(DcMotor.Direction.FORWARD);
        leftDCBack.setDirection(DcMotor.Direction.FORWARD);
        rightDCBack.setDirection(DcMotor.Direction.FORWARD);

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void init_loop() {
        telemetry.addData("Status", "Awaiting Start");
    }

    @Override
    public void start() {
        telemetry.addData("Status", "Started");
        runtime.reset();
    }

    @Override
    public void loop() {
        double r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
        double angleDC = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
        final double leftFrontSpeed = r*acc*Math.cos(angleDC) + gamepad1.right_stick_x;
        final double rightFrontSpeed = r*acc*Math.sin(angleDC) - gamepad1.right_stick_x;
        final double leftBackSpeed = r*acc*Math.sin(angleDC) + gamepad1.right_stick_x;
        final double rightBackSpeed = r*acc*Math.cos(angleDC) - gamepad1.right_stick_x;

        if(gamepad1.x) {
            if(slowMode) slowMode = false;
            else slowMode = true;
        }
        if(slowMode) acc = 0.3;
        else acc = 1.0;
        if(gamepad1.a) {
            if(isServoMaxed) isServoMaxed = false;
            else isServoMaxed = true;
        }
        if(isServoMaxed) servo.setPosition(1.0);
        else servo.setPosition(0.1);

        leftDCFront.setPower(leftFrontSpeed);
        rightDCFront.setPower(rightFrontSpeed);
        leftDCBack.setPower(leftBackSpeed);
        rightDCBack.setPower(rightBackSpeed);

        telemetry.addData("Status", "Looping");
        telemetry.addData("Runtime", runtime.toString() + " Milliseconds");
        telemetry.addData("Motors", "leftFront (%.2f), rightFront (%.2f), leftBack (%.2f), rightBack(%.2f)",
                leftDCFront, rightDCFront, leftDCBack, rightDCBack);
    }

    @Override
    public void stop() {
        telemetry.addData("Status", "Stopped");
    }
}