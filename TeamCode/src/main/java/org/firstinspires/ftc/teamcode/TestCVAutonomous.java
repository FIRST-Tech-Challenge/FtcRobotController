package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Autonomous Computer Vision Test")
public class TestCVAutonomous extends LinearOpMode {
    RobotClass robot;
    @Override
    public void runOpMode() throws InterruptedException {
        robot= new RobotClass(hardwareMap, telemetry, this);
        robot.wobbleGoalGrippyThingGrab();
        robot.openCVInnitShenanigans();

        waitForStart();

        new RobotClass.SkystoneDeterminationPipelineInnerBlueOuterRed();
        robot.analyze();
    }
}
