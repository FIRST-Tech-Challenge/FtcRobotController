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
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware.Actuators.ServoAdvanced;

import java.util.Timer;

@Config
public class Outtake {
    ServoAdvanced shoulderLeft;
    ServoAdvanced shoulderRight;
    ServoAdvanced armWrist;
    ServoAdvanced clawRotation;
    ServoAdvanced linkageLeft;
    ServoAdvanced linkageRight;
    public static double shoulderDown = 1;
    public static double shoulderFront = 0.9;
    public static double shoulderBack = 0;
    public static double shoulderBackSAMPLE = 0.15;
    public static double wristFront = 0.45;
    public static double wristBack = 0;
    public static double wristMiddle = 0.2;
    public static double clawRotationNormal = 0.45;
    public static double clawRotationReverse = 1;
    public static double linkageFront = 0.5;
    public static double linkageBack = 0.28;

    public Outtake(HardwareMap hardwareMap) {
        this.shoulderLeft = new ServoAdvanced(hardwareMap.get(Servo.class, "shoulderLeft"));
        this.shoulderRight = new ServoAdvanced(hardwareMap.get(Servo.class, "shoulderRight"));
        this.armWrist = new ServoAdvanced(hardwareMap.get(Servo.class, "armWrist"));
        this.clawRotation = new ServoAdvanced(hardwareMap.get(Servo.class, "clawRotation"));
        this.linkageLeft = new ServoAdvanced(hardwareMap.get(Servo.class, "linkageLeft"));
        this.linkageRight = new ServoAdvanced(hardwareMap.get(Servo.class, "linkageRight"));
    }

    public enum OuttakeState {
        RETRACT,    // pulls arm in
        FRONT,      // pushes arm out
        BACK,
        SAMPLE
    }

    public enum RotateState{
        NORMAL,
        FLIPPED
    }
    RotateState rotatePos = RotateState.NORMAL;
    OuttakeState outtakePos = OuttakeState.RETRACT;
    ElapsedTime outtakeTimer = new ElapsedTime();

    public Action OuttakeFrontSpecimen() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                double time = outtakeTimer.seconds();
                if (time > 0.3) {
                    if (outtakePos == OuttakeState.FRONT) {
                        shoulderLeft.setPosition(shoulderFront);
                        shoulderRight.setPosition(shoulderFront);
                        armWrist.setPosition(wristFront);
                        clawRotation.setPosition(clawRotationReverse);
                        linkageLeft.setPosition(linkageFront);
                        linkageRight.setPosition(linkageFront);

                        outtakePos = OuttakeState.RETRACT;
                    } else if (outtakePos == OuttakeState.RETRACT) {
                        shoulderLeft.setPosition(shoulderDown);
                        shoulderRight.setPosition(shoulderDown);
                        armWrist.setPosition(wristMiddle);
                        clawRotation.setPosition(clawRotationNormal);
                        linkageLeft.setPosition(linkageBack);
                        linkageRight.setPosition(linkageBack);

                        outtakePos = OuttakeState.FRONT;
                    }
                    outtakeTimer.reset();
                }
                return false;
            }
        };
    }

    public Action OuttakeBackSpecimen() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket Packet) {
                double time = outtakeTimer.seconds();
                if (time > 0.3) {
                    if (outtakePos == OuttakeState.BACK) {
                        shoulderLeft.setPosition(shoulderBack);
                        shoulderRight.setPosition(shoulderBack);
                        armWrist.setPosition(wristBack);
                        clawRotation.setPosition(clawRotationNormal);
                        linkageLeft.setPosition(linkageBack);
                        linkageRight.setPosition(linkageBack);

                        outtakePos = OuttakeState.RETRACT;
                    } else if (outtakePos == OuttakeState.RETRACT) {
                        shoulderLeft.setPosition(shoulderDown);
                        shoulderRight.setPosition(shoulderDown);
                        armWrist.setPosition(wristMiddle);
                        clawRotation.setPosition(clawRotationNormal);
                        linkageLeft.setPosition(linkageBack);
                        linkageRight.setPosition(linkageBack);

                        outtakePos = OuttakeState.BACK;
                    }
                    outtakeTimer.reset();
                }
                return false;
            }
        };
    }

    public Action OuttakeBackSample() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket Packet) {
                double time = outtakeTimer.seconds();
                if (time > 0.3) {
                    if (outtakePos == OuttakeState.SAMPLE) {
                        shoulderLeft.setPosition(shoulderDown);
                        shoulderRight.setPosition(shoulderDown);
                        armWrist.setPosition(wristMiddle);
                        clawRotation.setPosition(clawRotationNormal);
                        linkageLeft.setPosition(linkageBack);
                        linkageRight.setPosition(linkageBack);

                        outtakePos = OuttakeState.RETRACT;

                    } else if (outtakePos == OuttakeState.RETRACT) {
                        shoulderLeft.setPosition(shoulderBackSAMPLE);
                        shoulderRight.setPosition(shoulderBackSAMPLE);
                        armWrist.setPosition(wristBack);
                        clawRotation.setPosition(clawRotationNormal);
                        linkageLeft.setPosition(linkageBack);
                        linkageRight.setPosition(linkageBack);

                        outtakePos = OuttakeState.SAMPLE;
                    }
                    outtakeTimer.reset();
                }
                return false;
            }
        };
    }

    public Action OuttakeRetract(){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                shoulderLeft.setPosition(shoulderDown);
                shoulderRight.setPosition(shoulderDown);
                armWrist.setPosition(wristMiddle);
                clawRotation.setPosition(clawRotationNormal);
                linkageLeft.setPosition(linkageBack);
                linkageRight.setPosition(linkageBack);
                return false;
            }
        };
    }

    public Action clawRotate(){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if(rotatePos == RotateState.FLIPPED){
                    clawRotation.setPosition(clawRotationReverse);
                    rotatePos = RotateState.NORMAL;
                } else if(rotatePos == RotateState.NORMAL){
                    clawRotation.setPosition(clawRotationNormal);
                    rotatePos = RotateState.FLIPPED;
                }
                return false;
            }
        };
    }


    public Action sampleCollect() {
        return new SequentialAction(
                new InstantAction(() -> linkageLeft.setPosition(linkageBack)),
                new InstantAction(() -> linkageRight.setPosition(linkageBack)),
                new InstantAction(() -> clawRotation.setPosition(clawRotationNormal)),
                new InstantAction(() -> armWrist.setPosition(wristBack)),
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
                new InstantAction(()-> armWrist.setPosition(wristFront)),
                new SleepAction(0),
                new InstantAction(()->shoulderLeft.setPosition(shoulderFront)),
                new InstantAction(()->shoulderRight.setPosition(shoulderFront))
        );
    }

    public Action specimenCollect() {
        return new SequentialAction(
                new InstantAction(()->linkageLeft.setPosition(linkageFront)),
                new InstantAction(()->linkageRight.setPosition(linkageFront)),
                new InstantAction(()->clawRotation.setPosition(clawRotationNormal)),
                new InstantAction(()-> armWrist.setPosition(wristMiddle)),
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
                new InstantAction(()-> armWrist.setPosition(wristMiddle)),
                new SleepAction(0),
                new InstantAction(()->shoulderLeft.setPosition(shoulderBack)),
                new InstantAction(()->shoulderRight.setPosition(shoulderBack))
        );
    }
}

//    public Action OuttakeFront(){
//        return new Action(){
//            @Override
//            public boolean run(@NonNull TelemetryPacket Packet){
//                shoulderLeft.setPosition(shoulderFront);
//                shoulderRight.setPosition(shoulderFront);
//                armWrist.setPosition(wristFront);
//                clawRotation.setPosition(clawRotationReverse);
//                linkageLeft.setPosition(linkageFront);
//                linkageRight.setPosition(linkageFront);
//                return false;
//            }
//        };
//    }



