package org.firstinspires.ftc.teamcode.robots.ri2d2023;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robots.r2v2.util.Utils;
@Config("RI2D2023-INTAKE")
public class Intake {
    private Servo claw = null;
    private Servo clawArm = null;
    public static double CLAWOPEN = .25;
    public static double CLAWCLOSE = .6;
    public static double armSpeed = 50;
    private static int clawArmPosition;
    private Telemetry telemetry;
    private HardwareMap hardwareMap;
    public Intake(Telemetry telemetry, HardwareMap hardwareMap) {
        this.telemetry = telemetry;
        this.hardwareMap = hardwareMap;
    }
    public void telemetryOutput()
    {
        telemetry.addData("Claw Position \t", Utils.servoDenormalize(claw.getPosition()));
        telemetry.addData("Claw Arm Position \t", Utils.servoDenormalize(clawArm.getPosition()));
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
    public void moveClawArm(float position)
    {
        clawArmPosition = clawArmPosition + (int)(armSpeed*position);
        if(clawArmPosition < 1020)
            clawArmPosition = 1020;
        if(clawArmPosition > 1800)
            clawArmPosition = 1800;
        clawArm.setPosition(Utils.servoNormalize(clawArmPosition));
    }
    public double getClawPosition(){return claw.getPosition();}
}
