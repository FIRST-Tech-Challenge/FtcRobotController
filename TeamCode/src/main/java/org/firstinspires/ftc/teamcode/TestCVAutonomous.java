package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.Date;

@Autonomous(name = "Autonomous Computer Vision Test")
public class TestCVAutonomous extends LinearOpMode {
    RobotClass robot;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new RobotClass(hardwareMap, telemetry, this);
        robot.wobbleGoalGrippyThingGrab();
        robot.openCVInnitShenanigans();

        waitForStart();


        long startTime = new Date().getTime();
        long time = 0;

        while (time < 200 && opModeIsActive()) {
            time = new Date().getTime() - startTime;
            RobotClass.RingPosition position = robot.analyze();

            telemetry.addData("Position", position);
            telemetry.update();
        }

        robot.pause(5000);
    }


}

