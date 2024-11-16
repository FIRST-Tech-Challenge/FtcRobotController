package org.firstinspires.ftc.teamcode.Extension;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Extension {
    HardwareMap hardwareMap;
    Servo servoExtendLeft;
    Servo servoExtendRight;

    public Extension(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        servoExtendLeft = hardwareMap.get(Servo.class, "servoExtendLeft");
        servoExtendRight = hardwareMap.get(Servo.class, "servoExtendRight");
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
                    servoExtendLeft.setPosition(0.5);
                    servoExtendRight.setPosition(-0.5);

                }
                if (extensionPos == extensionState.EXTEND) {
                    servoExtendLeft.setPosition(-1);
                    servoExtendRight.setPosition(1);
                }
                return false;
            }
        };
    }
}