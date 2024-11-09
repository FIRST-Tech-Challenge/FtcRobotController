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
        CLOSE,
        OPEN
    }
    public Action servoClaw(clawState stateofClaw){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket Packet) {
                if (stateofClaw == clawState.OPEN) {
                    clawServo.setPosition(1);
                }
                if (stateofClaw == clawState.CLOSE) {
                    clawServo.setPosition(0);
                }
                // servo parameter -1, 0   0, 1
                return false;
            }
        };
    }
}

