package org.firstinspires.ftc.teamcode.fishlo.robot;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.SubSystem;

public class Claw extends SubSystem {
    private Servo claw;

    public Claw(Robot robot) {super(robot);}

    @Override
    public void init() {
        claw = robot.hardwareMap.servo.get("claw");
    }

    @Override
    public void handle() {

    }

    @Override
    public void stop() {

    }

}
