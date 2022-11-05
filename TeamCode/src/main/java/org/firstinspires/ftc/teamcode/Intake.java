package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Intake {
    private Servo arm;
    private Servo wrist;
    private Servo claw;

    Intake(HardwareMap hardwareMap, Telemetry telemetry){
        arm = hardwareMap.get(Servo.class, "arm");
        wrist = hardwareMap.get(Servo.class, "wrist");
        claw = hardwareMap.get(Servo.class, "claw");
    }

    public void runArm(double pos){
        arm.setPosition(pos);
    }

    public void runWrist(double pos){
        wrist.setPosition(pos);
    }

    public void runClaw(double pos){
        claw.setPosition(pos);
    }
}
