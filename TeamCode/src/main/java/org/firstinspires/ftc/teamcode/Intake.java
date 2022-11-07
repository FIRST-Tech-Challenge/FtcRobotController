package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Intake {
    private Servo armLeft;
    private Servo armRight;
    private Servo wrist;
    private Servo claw;

    WristMode wristMode;

    Intake(HardwareMap hardwareMap, Telemetry telemetry){
        armLeft = hardwareMap.get(Servo.class, "armLeft");
        armRight = hardwareMap.get(Servo.class, "armRight");
        wrist = hardwareMap.get(Servo.class, "wrist");
        claw = hardwareMap.get(Servo.class, "claw");
        wristMode = WristMode.MATCHED;
    }

    public enum WristMode{INDEPENDENT, MATCHED}

    public void runArm(double pos){
        armLeft.setPosition(pos);
        armRight.setPosition(1-pos);

        if (wristMode== WristMode.MATCHED){
            runWrist(pos);
        }
    }

    public void runArm(double pos, WristMode newWristMode){
        armLeft.setPosition(pos);
        armRight.setPosition(1-pos);

        if (newWristMode== WristMode.MATCHED){
            runWrist(pos);
        }
    }

    public void runWrist(double pos){
        wrist.setPosition(pos);
    }

    public void runClaw(double pos){
        claw.setPosition(pos);
    }

    public void changeWristMode(WristMode newWristMode){
        wristMode = newWristMode;
    }
}
