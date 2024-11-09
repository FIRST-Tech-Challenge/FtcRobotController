package org.firstinspires.ftc.teamcode.Arm;

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
        servoWrist = hardwareMap.get(Servo.class, "servoWrist");
        servoArmLeft = hardwareMap.get(Servo.class, "servoArmLeft");
        servoArmRight = hardwareMap.get(Servo.class, "servoArmRight");
    }

    public enum kwah {
        haut {
            @Override
            int servoone() {
                return 14;
            }
            @Override
            int servotwo() {
                return 12;
            }
            @Override
            int servothree(){
                return 18;
            }
        };
        abstract int servoone();
        abstract int servotwo();
        abstract int servothree();
    }

    public Action servoWrist(){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket Packet) {
//                servoWrist.setPosition(Hawk.OPEN);
//                servoArmLeft.setPosition(Hawk.CLOSE);
//                servoArmRight.setPosition(Hawk.WRIST);
                // servo parameter -1, 0   0, 1
                return false;
            }
        };
    }
}