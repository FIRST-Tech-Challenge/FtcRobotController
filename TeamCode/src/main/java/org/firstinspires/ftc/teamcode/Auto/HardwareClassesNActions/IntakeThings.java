package org.firstinspires.ftc.teamcode.Auto.HardwareClassesNActions;
import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.teleop.Values;

public class IntakeThings {
    private Servo clawPivot;
    private Servo intakeClaw;
    private Servo intakeElbow;
    private Servo wrist;

    public IntakeThings(HardwareMap hardwareMap) {
        intakeClaw = hardwareMap.get(Servo.class, "0");
        clawPivot = hardwareMap.get(Servo.class, "1");
        intakeElbow = hardwareMap.get(Servo.class, "3");
        wrist = hardwareMap.get(Servo.class, "2");
    }
    // within the intake class
    public class ClawUp implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            //not sure if the sleep action works
            intakeElbow.setPosition(Values.intakeElbowDown);
            new SleepAction(0.4);
            intakeClaw.setPosition(Values.intakeclawClose);
            new SleepAction(0.5);
            wrist.setPosition(Values.wristUp);
            clawPivot.setPosition(Values.MID_SERVO);
            intakeElbow.setPosition(Values.intakeElbowUp);
            return false;
        }
    }
    public Action clawClose() {
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
    public Action openClaw() {
        return new ClawWait();
    }
}