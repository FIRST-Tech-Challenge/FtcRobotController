package org.firstinspires.ftc.teamcode.Mechanisms.Arm;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Arm {
    HardwareMap hardwareMap;
    Servo servoWrist;
    Servo servoArmLeft;
    Servo servoArmRight;
    public Arm(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        this.servoWrist = hardwareMap.get(Servo.class, "wrist");
        this.servoArmLeft = hardwareMap.get(Servo.class, "armRight");
        this.servoArmRight = hardwareMap.get(Servo.class, "armLeft");
    }

    public enum armState {
        RETRACT,    // pulls arm in
        EXTEND      // pushes arm out
    }

    public Action servoArm(armState armPos){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket Packet) {
                if (armPos == armState.RETRACT) {
                    servoArmLeft.setPosition(0.5);
                    servoArmRight.setPosition(0.5);
                    servoWrist.setPosition(-0.5);

                }
                if (armPos == armState.EXTEND) {
                    servoArmLeft.setPosition(-1);
                    servoArmLeft.setPosition(-1);
                    servoWrist.setPosition(0);
                }
                return false;
            }
        };
    }
}