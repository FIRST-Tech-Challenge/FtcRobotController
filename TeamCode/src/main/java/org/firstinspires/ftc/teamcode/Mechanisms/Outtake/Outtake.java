package org.firstinspires.ftc.teamcode.Mechanisms.Outtake;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
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
        return new SequentialAction(
            new InstantAction(() -> shoulderLeft.setPosition(shoulderDown)),
            new SleepAction(0),
            new InstantAction(() -> shoulderRight.setPosition(shoulderDown)),
            new SleepAction(0),
            new InstantAction(() -> wristPitch.setPosition(wristPitchDown)),
            new SleepAction(0),
            new InstantAction(() -> wristRoll.setPosition(wristRollNormal)),
            new SleepAction(0),
            new InstantAction(() -> linkageLeft.setPosition(linkageMid)),
            new SleepAction(0),
            new InstantAction(() -> linkageRight.setPosition(linkageMid))
                    );
    }

    public Action sampleDeposit() {
        return new SequentialAction(
                new InstantAction(()->shoulderLeft.setPosition(shoulderUp)),
                new SleepAction(0),
                new InstantAction(()->shoulderRight.setPosition(shoulderUp)),
                new SleepAction(0),
                new InstantAction(()->wristPitch.setPosition(wristPitchUp)),
                new SleepAction(0),
                new InstantAction(()->wristRoll.setPosition(wristRollNormal)),
                new SleepAction(0),
                new InstantAction(()->linkageLeft.setPosition(linkageBack)),
                new SleepAction(0),
                new InstantAction(()->linkageRight.setPosition(linkageBack))
        );
    }

    public Action specimenCollect() {
        return new SequentialAction(
                new InstantAction(()->shoulderLeft.setPosition(shoulderFront)),
                new SleepAction(0),
                new InstantAction(()->shoulderRight.setPosition(shoulderFront)),
                new SleepAction(0),
                new InstantAction(()->wristPitch.setPosition(wristPitchMid)),
                new SleepAction(0),
                new InstantAction(()->wristRoll.setPosition(wristRollNormal)),
                new SleepAction(0),
                new InstantAction(()->linkageLeft.setPosition(linkageFront)),
                new SleepAction(0),
                new InstantAction(()->linkageRight.setPosition(linkageFront))
        );
    }

    public Action specimenDeposit() {
        return new SequentialAction(
                new InstantAction(()->shoulderLeft.setPosition(shoulderBack)),
                new SleepAction(0),
                new InstantAction(()->shoulderRight.setPosition(shoulderBack)),
                new SleepAction(0),
                new InstantAction(()->wristPitch.setPosition(wristPitchMid)),
                new SleepAction(0),
                new InstantAction(()->wristRoll.setPosition(wristRollReverse)),
                new SleepAction(0),
                new InstantAction(()->linkageLeft.setPosition(linkageBack)),
                new SleepAction(0),
                new InstantAction(()->linkageRight.setPosition(linkageBack))
        );
    }
}
