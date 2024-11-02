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
    public Action clawClose(){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket Packet) {
                servoLeft.setPosition(0);
                servoRight.setPosition(0);
                // set position
                return false;
            }
        };
    }
}

