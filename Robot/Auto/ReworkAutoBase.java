package org.firstinspires.ftc.teamcode.rework.Robot.Auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.rework.Robot.Robot;

public abstract class ReworkAutoBase extends LinearOpMode {
    protected Robot robot;

    protected void initRobot() {
        robot = new Robot(hardwareMap, telemetry, this);

        robot.initModules();
    }

    protected void purePursuitMove() {
        // TODO
    }

    protected void moveToPoint() {
        // TODO
    }
}
