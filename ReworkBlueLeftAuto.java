package org.firstinspires.ftc.teamcode.rework;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.rework.Robot.Auto.ReworkAutoBase;

@Autonomous
public class ReworkBlueLeftAuto extends ReworkAutoBase {
    public void runOpMode() {
        initRobot();

        waitForStart();

        robot.startModules();

        purePursuitMove();
    }
}
