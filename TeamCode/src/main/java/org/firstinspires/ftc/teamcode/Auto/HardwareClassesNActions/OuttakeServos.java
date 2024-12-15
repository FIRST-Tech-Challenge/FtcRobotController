package org.firstinspires.ftc.teamcode.Auto.HardwareClassesNActions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.teleop.Values;

public class OuttakeServos {
    public Servo outtakeClaw = null;
    public Servo outtakeElbow = null;

    public OuttakeServos (HardwareMap hardwareMap) {
        outtakeClaw = hardwareMap.get(Servo.class, "6");
        outtakeElbow = hardwareMap.get(Servo.class,"7");
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
            new SleepAction(.4);
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
}
