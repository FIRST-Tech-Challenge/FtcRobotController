package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
public class Backdrop {
    private LinearOpMode aggregate;

    public static double BACKDROP_DEFAULT_POS = 0;
    public static double CLAW_DEFAULT_POS = 0.7;
    public static double BACKDROP_FINAL_POS = 0.4;
    public static double CLAW_FINAL_POS = 0;

    private Servo backdrop;
    private Servo claw;

    public Backdrop(LinearOpMode aggregate) {
        backdrop = aggregate.hardwareMap.get(Servo.class, "backdrop");
        claw = aggregate.hardwareMap.get(Servo.class, "claw");

        backdrop.setPosition(BACKDROP_DEFAULT_POS);
        claw.setPosition(CLAW_DEFAULT_POS);

        this.aggregate = this.aggregate;
    }

    public void moveUp() {
        backdrop.setPosition(BACKDROP_FINAL_POS);
    }

    public void moveDown() {
        backdrop.setPosition(BACKDROP_DEFAULT_POS);
    }

    public void release() {
        claw.setPosition(CLAW_FINAL_POS);
    }
}