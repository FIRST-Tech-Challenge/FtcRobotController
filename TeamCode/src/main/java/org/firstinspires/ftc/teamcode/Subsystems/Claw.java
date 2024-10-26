package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Claw {
    public Servo claw;
    public final Gamepad gamepad2;
    public HardwareMap hardwareMap;
    public double OPENED = 0;
    public double CLOSED = 0.8;
    public Claw(OpMode opMode) {
        this.hardwareMap = opMode.hardwareMap;
        this.gamepad2 = opMode.gamepad2;

        claw = (Servo) hardwareMap.get("claw");

        claw.setDirection(Servo.Direction.FORWARD);
        clawServo(OPENED);
    }
    public void teleOp() {
        if (gamepad2.left_bumper) clawServo(OPENED);
        else if (gamepad2.right_bumper) clawServo(CLOSED);
    }
    public void clawServo(double position) {
        claw.setPosition(position);
    }
}