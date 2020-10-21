package org.firstinspires.ftc.teamcode.fishlo.v1.fishlo.robot;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.SubSystem;

public class Claw extends SubSystem {
    private Servo claw;
    public static final double CLAW_HOME = 0.0;
    public static final double CLAW_MAX = 0.5;

    public Claw(Robot robot) {
        super(robot);
    }

    @Override
    public void init() {
        claw = robot.hardwareMap.servo.get("claw");
    }

    @Override
    public void handle() {
        if (robot.gamepad1.x) {
            grab();
        }
        if (robot.gamepad1.b) {
            reset();
        }
    }

    @Override
    public void stop() {
        reset();
    }

    public void grab() {
        claw.setPosition(CLAW_MAX);
    }

    public void reset() {
        claw.setPosition(CLAW_HOME);
    }

}
