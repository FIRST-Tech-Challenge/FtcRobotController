package org.firstinspires.ftc.teamcode.rework.Robot.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.rework.Robot.ReworkRobot;

public abstract class ReworkAutoBase extends LinearOpMode {
    protected ReworkRobot robot;

    protected void initRobot() {
        robot = new ReworkRobot(hardwareMap, telemetry, this);

        robot.initModules();
    }

    protected void purePursuitMove() {
        // TODO
    }

    protected void moveToPoint() {
        // TODO
    }
}
