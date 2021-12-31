package org.firstinspires.ftc.masters;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.Date;

@Autonomous(name = "Red Carousel", group ="competition")
public class JevilsRedCarousel extends LinearOpMode {
    RobotClass robot;

    @Override
    public void runOpMode() {

        robot = new RobotClass(hardwareMap, telemetry, this);
        robot.openCVInnitShenanigans();
        FreightFrenzyComputerVisionRedHub.SkystoneDeterminationPipeline.FreightPosition freightLocation = null;
        freightLocation = robot.analyze();
        waitForStart();

        long startTime = new Date().getTime();
        long time = 0;

        while (time < 1000 && opModeIsActive()) {
            time = new Date().getTime() - startTime;
            freightLocation = robot.analyze();

            telemetry.addData("Position", freightLocation);
            telemetry.update();
        }
        if (freightLocation == FreightFrenzyComputerVisionRedHub.SkystoneDeterminationPipeline.FreightPosition.MIDDLE) {

        }
        robot.forward(0.4, 1.6);
        robot.turnToHeadingSloppy(.4,-45,-15);
        robot.forward(.3,.7);

        if (freightLocation == FreightFrenzyComputerVisionRedHub.SkystoneDeterminationPipeline.FreightPosition.LEFT) {
            robot.dumpFreightBottom();
        } else if (freightLocation == FreightFrenzyComputerVisionRedHub.SkystoneDeterminationPipeline.FreightPosition.MIDDLE) {
            robot.dumpFreightMiddle();
        } else if (freightLocation == FreightFrenzyComputerVisionRedHub.SkystoneDeterminationPipeline.FreightPosition.RIGHT) {
            robot.dumpFreightTop();
        }

        robot.forward(.3,-2.5);
        robot.strafeLeft(.4,.45);
        robot.forward(.2,-.4);

        robot.jevilTurnCarousel(.5,2);
        robot.forward(0.3, 1);
        robot.turnToHeadingSloppy(.3,-90,-10);
        robot.forward(0.3, -1.5);

        robot.strafeRight(0.3, 0.5);
//        robot.parkRed();

    }
}

