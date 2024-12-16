package org.firstinspires.ftc.teamcode.Auto.HardwareClassesNActions;
import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.teleop.Values;

public class Servos {
    private Servo clawPivot;
    private Servo intakeClaw;
    private Servo intakeElbow;
    private Servo wrist;
    public Servo outtakeClaw;
    public Servo outtakeElbow;
    private Servo intakeSlide1;
    private Servo intakeSlide2;

    public Servos(HardwareMap hardwareMap) {
        intakeClaw = hardwareMap.get(Servo.class, "0");
        clawPivot = hardwareMap.get(Servo.class, "1");
        intakeElbow = hardwareMap.get(Servo.class, "3");
        wrist = hardwareMap.get(Servo.class, "2");
        intakeSlide1 = hardwareMap.get(Servo.class, "5");
        intakeSlide2 = hardwareMap.get(Servo.class,"4");
        outtakeClaw = hardwareMap.get(Servo.class, "6");
        outtakeElbow = hardwareMap.get(Servo.class,"7");
    }
    // within the intake class
    public class GrabWaitSequence implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            //not sure if the sleep action works
            intakeElbow.setPosition(Values.intakeElbowWait);
            intakeClaw.setPosition(Values.intakeClawOpen);
            wrist.setPosition(Values.wristDown);
            outtakeClaw.setPosition(Values.outakeclawOpen);
            outtakeElbow.setPosition(Values.outtakeElbowDown);
            return false;
        }
    }
    public Action GrabWaitSequence() {
        return new GrabWaitSequence();
    }

    public class GrabNTransferSequence implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            //not sure if the sleep action works
            intakeElbow.setPosition(Values.intakeElbowDown);
            new SleepAction(0.5);
            intakeClaw.setPosition(Values.intakeclawClose);
            new SleepAction(0.4);
            wrist.setPosition(Values.wristUp);
            clawPivot.setPosition(Values.MID_SERVO);
            intakeElbow.setPosition(Values.intakeElbowUp);
            intakeSlide1.setPosition(Values.slide1wait);
            intakeSlide2.setPosition(Values.slide2wait);
            //slidesIn();
            new SleepAction(1.2);
            outtakeClaw.setPosition(Values.outtakeClawClose);
            new SleepAction(0.400);
            intakeClaw.setPosition(Values.intakeClawOpen);
            intakeSlide1.setPosition(Values.slide1wait-0.1);
            intakeSlide2.setPosition(Values.slide2wait+0.1);
            new SleepAction(.200);
            intakeElbow.setPosition(Values.intakeElbowWait);
            outtakeElbow.setPosition(Values.outtakeElbowUp);
            return false;
        }
    }
    public Action grabNTransfer() {
        return new GrabNTransferSequence();
    }

    public class ClawUp implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            //not sure if the sleep action works
            intakeElbow.setPosition(Values.intakeElbowDown);
            wrist.setPosition(Values.wristDown);
            new SleepAction(0.4);
            intakeClaw.setPosition(Values.intakeclawClose);
            new SleepAction(0.5);
            wrist.setPosition(Values.wristUp);
            clawPivot.setPosition(Values.MID_SERVO);
            intakeElbow.setPosition(Values.intakeElbowUp);
            return false;
        }
    }
    public Action clawGrab() {
        return new ClawUp();
    }

    public class ClawWait implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            //the same as teleop cirle, but all the intake stuff
            intakeElbow.setPosition(Values.intakeElbowWait);
            intakeClaw.setPosition(Values.intakeClawOpen);
            wrist.setPosition(Values.wristDown);
            return false;
        }
    }
    public Action openWait() {
        return new ClawWait();
    }
    public class GetOutOfTheWay implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {

            intakeClaw.setPosition(Values.intakeClawOpen);
            intakeElbow.setPosition(Values.intakeElbowWait);
            return false;
        }
    }
    public Action getOuttaTheWay() {
        return new GetOutOfTheWay();
    }

    public class OuttakeClose implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            outtakeClaw.setPosition(Values.outtakeClawClose);
            new SleepAction(1.6);
            return false;
        }
    }
    public Action OuttakeClose() {
        return new OuttakeClose();
    }

    public class OuttakeOpen implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            new SleepAction(3);
            outtakeClaw.setPosition(Values.outakeclawOpen);
            return false;
        }
    }
    public Action outtakeOpen() {
        return new OuttakeOpen();
    }

    public class OuttakeFlat implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            outtakeElbow.setPosition(Values.outtakeElbowFlat);
            return false;
        }
    }
    public Action outtakeFlat() {
        return new OuttakeFlat();
    }

    public class OuttakeUp implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            outtakeElbow.setPosition(Values.outtakeElbowUp);
            return false;
        }
    }
    public Action outtakeUp() {
        return new OuttakeUp();
    }

    public class OuttakeDown implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            outtakeElbow.setPosition(Values.outtakeElbowDown);
            return false;
        }
    }
    public Action outtakedown() {
        return new OuttakeDown();
    }

    public class SlideOut implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            intakeSlide1.setPosition(Values.slide1out);
            intakeSlide2.setPosition(Values.slide2out);
            return false;
        }
    }
    public Action slidesOut() {
        return new SlideOut();
    }

    public class SlideIn implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            intakeSlide1.setPosition(Values.slide1in);
            intakeSlide2.setPosition(Values.slide2in);
            return false;
        }
    }
    public Action slidesIn() {
        return new SlideIn();
    }

    public class SlideWait implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            intakeSlide1.setPosition(Values.slide1wait);
            intakeSlide2.setPosition(Values.slide2wait);
            return false;
        }
    }
    public Action slidesWait() {
        return new SlideWait();
    }

    public class SlideOutOfTheWay implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {

            intakeSlide1.setPosition(Values.slide1wait);
            intakeSlide1.setPosition(Values.slide2wait);
            intakeSlide1.setPosition(Values.slide1wait-0.1);
            intakeSlide2.setPosition(Values.slide2wait+0.1);
            return false;
        }
    }
    public Action slideOuttaTheWay() {
        return new SlideOutOfTheWay();
    }



    public class ElbowUp implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {

            intakeElbow.setPosition(Values.intakeElbowUp);
            return false;
        }
    }
    public Action elbowUp() {
        return new ElbowUp();
    }
}