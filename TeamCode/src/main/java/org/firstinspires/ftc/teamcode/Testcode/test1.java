package org.firstinspires.ftc.teamcode.Testcode;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;

@TeleOp (name = "test1", group = "TeleOp")
public class test1 extends OpMode {

    HardwarePushbot robot = new HardwarePushbot();

    private double MOTOR_SPEED;

    private DcMotor backLeft;
    private DcMotor backRight;
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor carousel;



    public void init() {
        robot.init(hardwareMap);

    }

    public void loop() {

        move();

        carousel.setPower(gamepad2.right_stick_x);



    }

    public void move() {

        MOTOR_SPEED = gamepad1.left_stick_y;

        backLeft.setPower(MOTOR_SPEED);
        frontLeft.setPower(MOTOR_SPEED);
        frontRight.setPower(MOTOR_SPEED);
        backRight.setPower(MOTOR_SPEED);

    }
}