package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class ServoTester extends OpMode {
    //declare all motors
    Servo rightClaw;
    Servo leftClaw;

    @Override
    public void init() {
        telemetry.addData("Status", "Initializing");

        //assign all motors
        rightClaw = hardwareMap.get(Servo.class, "RightClaw");
        leftClaw =  hardwareMap.get(Servo.class, "LeftClaw");

        //tells user that the motors have been initialized
        telemetry.addData("Status", "Initialized!");
    }

    @Override
    public void loop() {
        if(gamepad1.dpad_right) {
            rightClaw.setPosition(1);
            leftClaw.setPosition(1);
        }
        if(gamepad1.dpad_left) {
            rightClaw.setPosition(0);
            leftClaw.setPosition(0);
        }
    }
}
