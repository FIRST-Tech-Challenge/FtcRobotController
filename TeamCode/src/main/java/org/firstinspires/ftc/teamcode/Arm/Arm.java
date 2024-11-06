package org.firstinspires.ftc.teamcode.Arm;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.Enumeration;

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

    public enum Hawk {
        wrist(10),
        open(12),
        close(15);

        private final int tu;
        Hawk(int value) {
            this.tu = value;
        }
    }


    public Action servoWrist(){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket Packet) {
                //end pos of claw finding
//                servoWrist.setPosition(Hawk.open);
//                servoArmLeft.setPosition(Hawk.close);
//                servoArmRight.setPosition(Hawk.wrist);
                // parameter -1, 0   0, 1
                return false;
            }
        };
    }


}
