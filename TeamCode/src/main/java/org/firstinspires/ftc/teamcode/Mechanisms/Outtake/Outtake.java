package org.firstinspires.ftc.teamcode.Mechanisms.Outtake;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Hardware.Actuators.ServoAdvanced;

@Config
public class Outtake {
    ServoAdvanced shoulderLeft;
    ServoAdvanced shoulderRight;
    ServoAdvanced armWrist;
    ServoAdvanced clawRotation;
    ServoAdvanced linkageLeft;
    ServoAdvanced linkageRight;
    public static double shoulderUp = 0;
    public static double shoulderDown = 0.2;
    public static double shoulderFront = 0.1;
    public static double shoulderBack = 0;
    public static double wristUp = 0.3;
    public static double wristDown = 0.1;
    public static double wristNeutral = 0;
    public static double clawRotationNormal = 0;
    public static double clawRotationReverse = 0.6;
    public static double linkageFront = 0.1;
    public static double linkageBack = 0;
    public static double linkageMid = 0.2;

    public Outtake(HardwareMap hardwareMap) {
        this.shoulderLeft = new ServoAdvanced(hardwareMap.get(Servo.class, "shoulderLeft"));
        this.shoulderRight = new ServoAdvanced(hardwareMap.get(Servo.class, "shoulderRight"));
        this.armWrist = new ServoAdvanced(hardwareMap.get(Servo.class, "armWrist"));
        this.clawRotation = new ServoAdvanced(hardwareMap.get(Servo.class, "clawRotation"));
        this.linkageLeft = new ServoAdvanced(hardwareMap.get(Servo.class, "linkageLeft"));
        this.linkageRight = new ServoAdvanced(hardwareMap.get(Servo.class, "linkageRight"));
    }

    public Action OuttakeRetract(){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket Packet) {
                shoulderLeft.setPosition(shoulderDown);
                shoulderRight.setPosition(shoulderDown);
                armWrist.setPosition(wristDown);
                clawRotation.setPosition(clawRotationNormal);
                linkageLeft.setPosition(linkageMid);
                linkageRight.setPosition(linkageMid);
                return false;
            }
        };
    }

    public Action OuttakeFront(){
        return new Action(){
            @Override
            public boolean run(@NonNull TelemetryPacket Packet){
                shoulderLeft.setPosition(shoulderFront);
                shoulderRight.setPosition(shoulderFront);
                armWrist.setPosition(wristUp);
                clawRotation.setPosition(clawRotationNormal);
                linkageLeft.setPosition(linkageFront);
                linkageRight.setPosition(linkageFront);
                return false;
            }
        };
    }









    // BELOW MOVE TO ROBOT

    public Action sampleCollect() {
        return new SequentialAction(
            new InstantAction(() -> linkageLeft.setPosition(linkageMid)),
            new InstantAction(() -> linkageRight.setPosition(linkageMid)),
            new InstantAction(() -> clawRotation.setPosition(clawRotationNormal)),
            new InstantAction(() -> armWrist.setPosition(wristDown)),
            new SleepAction(0),
            new InstantAction(() -> shoulderLeft.setPosition(shoulderDown)),
            new InstantAction(() -> shoulderRight.setPosition(shoulderDown))
        );
    }

    public Action sampleDeposit() {
        return new SequentialAction(
                new InstantAction(()->linkageLeft.setPosition(linkageBack)),
                new InstantAction(()->linkageRight.setPosition(linkageBack)),
                new InstantAction(()->clawRotation.setPosition(clawRotationNormal)),
                new InstantAction(()-> armWrist.setPosition(wristUp)),
                new SleepAction(0),
                new InstantAction(()->shoulderLeft.setPosition(shoulderUp)),
                new InstantAction(()->shoulderRight.setPosition(shoulderUp))
        );
    }

    public Action specimenCollect() {
        return new SequentialAction(
                new InstantAction(()->linkageLeft.setPosition(linkageFront)),
                new InstantAction(()->linkageRight.setPosition(linkageFront)),
                new InstantAction(()->clawRotation.setPosition(clawRotationNormal)),
                new InstantAction(()-> armWrist.setPosition(wristNeutral)),
                new SleepAction(0),
                new InstantAction(()->shoulderLeft.setPosition(shoulderFront)),
                new InstantAction(()->shoulderRight.setPosition(shoulderFront))
        );
    }

    public Action specimenDeposit() {
        return new SequentialAction(
                new InstantAction(()->linkageLeft.setPosition(linkageBack)),
                new InstantAction(()->linkageRight.setPosition(linkageBack)),
                new InstantAction(()->clawRotation.setPosition(clawRotationReverse)),
                new InstantAction(()-> armWrist.setPosition(wristNeutral)),
                new SleepAction(0),
                new InstantAction(()->shoulderLeft.setPosition(shoulderBack)),
                new InstantAction(()->shoulderRight.setPosition(shoulderBack))
        );
    }
}
