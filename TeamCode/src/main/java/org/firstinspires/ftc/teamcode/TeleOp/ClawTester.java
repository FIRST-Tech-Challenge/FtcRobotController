package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp

public class ClawTester extends OpMode {

    Servo rightClaw;
    Servo leftClaw;

    @Override
    public void init() {
        telemetry.addData("Status", "Initializing");

        rightClaw = hardwareMap.get(Servo.class, "RightClaw");
        leftClaw =  hardwareMap.get(Servo.class, "LeftClaw");

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void loop() {
        if(gamepad1.b) {    // Close claw
            rightClaw.setPosition(1);
            leftClaw.setPosition(0);
        }
        if(gamepad1.x) {    // Open claw
            rightClaw.setPosition(0.5);
            leftClaw.setPosition(0.5);
        }
    }
}