package org.firstinspires.ftc.teamcode.Mechanisms.Outtake;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Hardware.Actuators.ServoAdvanced;

@Config
public class Outtake {
    ServoAdvanced shoulderLeft;
    ServoAdvanced shoulderRight;
    ServoAdvanced wristPitch;
    ServoAdvanced wristRoll;
    ServoAdvanced linkageLeft;
    ServoAdvanced linkageRight;
    public static double shoulderUp = 0;
    public static double shoulderDown = 0;
    public static double shoulderFront = 0;
    public static double shoulderBack = 0;
    public static double wristPitchUp = 0;
    public static double wristPitchDown = 0;
    public static double wristPitchMid = 0;
    public static double wristRollNormal = 0;
    public static double wristRollReverse = 0;
    public static double linkageFront = 0;
    public static double linkageBack = 0;
    public static double linkageMid = 0;

    public Outtake(HardwareMap hardwareMap) {
        this.shoulderLeft = new ServoAdvanced(hardwareMap.get(Servo.class, "shoulderLeft"));
        this.shoulderRight = new ServoAdvanced(hardwareMap.get(Servo.class, "shoulderRight"));
        this.wristPitch = new ServoAdvanced(hardwareMap.get(Servo.class, "wristPitch"));
        this.wristRoll = new ServoAdvanced(hardwareMap.get(Servo.class, "wristRoll"));
        this.linkageLeft = new ServoAdvanced(hardwareMap.get(Servo.class, "linkageLeft"));
        this.linkageRight = new ServoAdvanced(hardwareMap.get(Servo.class, "linkageRight"));
    }

    public Action sampleCollect() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket Packet) {
                shoulderLeft.setPosition(shoulderDown);
                shoulderRight.setPosition(shoulderDown);
                wristPitch.setPosition(wristPitchDown);
                wristRoll.setPosition(wristRollNormal);
                linkageLeft.setPosition(linkageMid);
                linkageRight.setPosition(linkageMid);
                return false;
            }
        };
    }

    public Action sampleDeposit() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket Packet) {
                shoulderLeft.setPosition(shoulderUp);
                shoulderRight.setPosition(shoulderUp);
                wristPitch.setPosition(wristPitchUp);
                wristRoll.setPosition(wristRollNormal);
                linkageLeft.setPosition(linkageBack);
                linkageRight.setPosition(linkageBack);
                return false;
            }
        };
    }

    public Action specimenCollect() {
        return new Action() {
            public boolean run(@NonNull TelemetryPacket Packet) {
                shoulderLeft.setPosition(shoulderFront);
                shoulderRight.setPosition(shoulderFront);
                wristPitch.setPosition(wristPitchMid);
                wristRoll.setPosition(wristRollNormal);
                linkageLeft.setPosition(linkageFront);
                linkageRight.setPosition(linkageFront);
                return false;
            }
        };
    }

    public Action specimenDeposit() {
        return new Action() {
            public boolean run(@NonNull TelemetryPacket Packet) {
                shoulderLeft.setPosition(shoulderBack);
                shoulderRight.setPosition(shoulderBack);
                wristPitch.setPosition(wristPitchMid);
                wristRoll.setPosition(wristRollReverse);
                linkageLeft.setPosition(linkageBack);
                linkageRight.setPosition(linkageBack);
                return false;
            }
        };
    }
}
