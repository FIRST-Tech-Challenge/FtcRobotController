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
    public enum hawk {
        two {
            @Override
            int openClaw() {
                return 14;
            }
            @Override
            int closeClaw() {
                return 12;
            }
        };
        abstract int openClaw();
        abstract int closeClaw();
    }


    public Action claw(){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket Packet) {
                servoLeft.setPosition(hawk.two.closeClaw());
                servoRight.setPosition(hawk.two.openClaw());
                // set position
                return false;
            }
        };
    }
}

