package org.firstinspires.ftc.teamcode.rework;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.rework.AutoTools.ReworkAutoBase;

@Autonomous
public class BlueLeftAuto extends ReworkAutoBase {
    public void runOpMode() {
        initRobot();

        waitForStart();

        robot.startModules();

        purePursuitMove();
    }
}
