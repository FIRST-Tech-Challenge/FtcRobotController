package org.firstinspires.ftc.teamcode.Mechanisms.Extension;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Extension {
    HardwareMap hardwareMap;
    Servo servoExtendLeft;
    Servo servoExtendRight;
    public static double extendPos = 0.5;
    public static double retractPos = 1;
    public Extension(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        servoExtendLeft = hardwareMap.get(Servo.class, "leftExtension");
        servoExtendRight = hardwareMap.get(Servo.class, "rightExtension");
    }

    public enum extensionState {
        RETRACT, //pull extension back
        EXTEND   //push extension forward
    }

    public Action servoExtension(extensionState extensionPos) {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket Packet) {
                if (extensionPos == extensionState.RETRACT) {
                    servoExtendLeft.setPosition(extendPos);
                    servoExtendRight.setPosition(-extendPos);

                }
                if (extensionPos == extensionState.EXTEND) {
                    servoExtendLeft.setPosition(retractPos);
                    servoExtendRight.setPosition(-retractPos);
                }
                return false;
            }
        };
    }
}