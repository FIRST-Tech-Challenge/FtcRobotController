package org.firstinspires.ftc.teamcode.auto;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class RobotServos {
    private Servo rotateArm;
    private Servo bottomClaw;
    private Servo rotateBClaw;
    private Servo flipTClaw;
    private Servo rotateTClaw;
    private Servo topClaw;

    public RobotServos(HardwareMap hardwareMap) {
        rotateArm    = hardwareMap.get(Servo.class, "rotateArm");
        bottomClaw   = hardwareMap.get(Servo.class, "bottomClaw");
        rotateBClaw  = hardwareMap.get(Servo.class, "rotateBClaw");
        flipTClaw    = hardwareMap.get(Servo.class, "flipTClaw");
        rotateTClaw  = hardwareMap.get(Servo.class, "rotateTClaw");
        topClaw      = hardwareMap.get(Servo.class, "topClaw");
    }


    public Action setServoPosition(Servo servo, double targetPos) {
        return new Action() {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    servo.setPosition(targetPos);
                    initialized = true;
                }
                return false;
            }
        };
    }

    public Action moveBottomClaw(double position) {
        return setServoPosition(bottomClaw, position);
    }
    public Action moveTopClaw(double position) {
        return setServoPosition(topClaw, position);
    }
    public Action moveRotateArm(double position) {
        return setServoPosition(rotateArm, position);
    }
    public Action moveRotateBClaw(double position) {
        return setServoPosition(rotateBClaw, position);
    }
    public Action moveFlipTClaw(double position) {
        return setServoPosition(flipTClaw, position);
    }
    public Action moveRotateTClaw(double position) {
        return setServoPosition(rotateTClaw, position);
    }
}