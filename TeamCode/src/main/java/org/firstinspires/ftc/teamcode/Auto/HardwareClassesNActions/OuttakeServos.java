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
            outtakeElbow.setPosition(Values.outtakeElbowUp);
            return false;
        }
    }
    public Action OuttakeOpen() {
        return new OuttakeClose();
    }
    public class OuttakeOpen implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            outtakeClaw.setPosition(Values.outakeclawOpen);
            return false;
        }
    }
    public Action outtakeOpen() {
        return new OuttakeOpen();
    }
}
