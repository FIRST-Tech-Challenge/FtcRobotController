package org.firstinspires.ftc.teamcode.robots.ri2d2023;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Intake {
    private Servo claw = null;
    private Servo clawArm = null;
    public static double CLAWOPEN = .5;
    public static double CLAWCLOSE = .2;
    private Telemetry telemetry;
    private HardwareMap hardwareMap;
    public Intake(Telemetry telemetry, HardwareMap hardwareMap) {
        this.telemetry = telemetry;
        this.hardwareMap = hardwareMap;
    }
    public void telemetryOutput()
    {
        telemetry.addData("Claw Position \t", claw.getPosition());
    }
    public void init()
    {
        claw = this.hardwareMap.get(Servo.class, "clawServo");
        clawArm = this.hardwareMap.get(Servo.class, "clawArmServo");
    }
    public void clawOpen() {
        claw.setPosition(CLAWOPEN);
    }
    public void clawClose() {
        claw.setPosition(CLAWCLOSE);
    }
    public void moveClawArm(int position)
    {
        clawArm.setPosition(position);
    }
    public double getClawPosition(){return claw.getPosition();}
}
