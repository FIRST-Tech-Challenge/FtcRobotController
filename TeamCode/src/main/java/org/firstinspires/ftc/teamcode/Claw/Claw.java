package org.firstinspires.ftc.teamcode.Claw;
import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


public class Claw {
    HardwareMap hardwareMap;
    Servo clawServo;


    public Claw(HardwareMap hardwareMap){
        this.hardwareMap = hardwareMap;
        clawServo = hardwareMap.get(Servo.class, "clawServo");

    }

    public enum clawState {
        CLOSE, //spins to close servo, should only be closed enough to hold piece
        OPEN   //spins to have nearly fully open servo
    }
    public Action servoClaw(clawState clawPos){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket Packet) {
                if (clawPos == clawState.OPEN) {
                    clawServo.setPosition(1);
                }
                if (clawPos == clawState.CLOSE) {
                    clawServo.setPosition(0);
                }
                // servo parameter -1, 0   0, 1
                return false;
            }
        };
    }
}

