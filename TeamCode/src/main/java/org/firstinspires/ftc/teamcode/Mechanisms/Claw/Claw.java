package org.firstinspires.ftc.teamcode.Mechanisms.Claw;
import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


@Config
public class Claw {
    HardwareMap hardwareMap;
    Servo clawServo;

    public static double clawOpen = 0;
    public static double clawClosed = 0;
    public Claw(HardwareMap hardwareMap){
        this.hardwareMap = hardwareMap;
        this.clawServo = hardwareMap.get(Servo.class, "clawServo");

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
                    clawServo.setPosition(clawOpen);
                }
                if (clawPos == clawState.CLOSE) {
                    clawServo.setPosition(clawClosed);
                }
                // servo parameter -1, 0   0, 1
                return false;
            }
        };
    }
}

