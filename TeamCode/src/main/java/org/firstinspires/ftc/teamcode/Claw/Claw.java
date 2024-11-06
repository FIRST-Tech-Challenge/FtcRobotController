package org.firstinspires.ftc.teamcode.Claw;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Claw {
    Servo servoLeft;
    Servo servoRight;
    HardwareMap hardwareMap;

    public Claw(HardwareMap hardwareMap){
        this.hardwareMap = hardwareMap;
        servoLeft = hardwareMap.get(Servo.class, "clawServoLeft");
        servoRight = hardwareMap.get(Servo.class, "clawServoRight");
    }

    public Action clawOpen(){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket Packet) {
                servoLeft.setPosition(0.25);
                servoRight.setPosition(-0.25);
                // TEST POSITION
                return false;
            }
        };
    }

    public enum tua {
        OPEN(12),
        CLOSE(15),
        MIDDLE(18);

        private final int value;

        tua(int value) {
            this.value = value;
        }

        public int getValue() {
            return value;
        }
    }


    public Action clawClose(){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket Packet) {
//                servoLeft.setPosition(tua.CLOSE);
//                servoRight.setPosition(tua.OPEN);
                // set position
                return false;
            }
        };
    }
}

