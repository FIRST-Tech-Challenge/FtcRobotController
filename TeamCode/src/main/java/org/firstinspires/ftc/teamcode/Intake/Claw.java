package org.firstinspires.ftc.teamcode.Intake;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ServoManage;

public class Claw {
    private ServoManage servo;
    private Telemetry telemetry;

    public Claw(HardwareMap hardwareMap, Telemetry telemetryGet) {
        servo = new ServoManage(hardwareMap, telemetry, "servo1");

        telemetry = telemetryGet;
    }

    public void toggleClaw(boolean toggle, double closePos, double openPos) {
        if (toggle) {
            if (servo.getPos() > 0.15) {
                servo.servoPositionX(closePos);
            }
            else {
                servo.servoPositionX(openPos);
            }
        }
        telemetry.addData("servo pos", servo.getPos());
        telemetry.update();
    }

    public void openOrCloseClaw(boolean toOpen, boolean toClose, double closePos, double openPos) {
        if (toOpen) {
            servo.servoPositionX(openPos);
        }
        else if (toClose) {
            servo.servoPositionX(closePos);
        }
        telemetry.addData("servo pos", servo.getPos());
        telemetry.update();
    }
}