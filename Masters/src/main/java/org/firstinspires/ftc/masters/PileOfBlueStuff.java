package org.firstinspires.ftc.masters;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.Date;

@Autonomous(name = "warehouse blue", group="competition")
public class PileOfBlueStuff extends LinearOpMode {
    RobotClass robot;

    @Override
    public void runOpMode() {
        robot = new RobotClass(hardwareMap,telemetry,this);
        robot.openCVInnitShenanigans("blue");
        FreightFrenzyComputerVisionRedHub.SkystoneDeterminationPipeline.FreightPosition freightLocation = null;
        freightLocation = robot.analyze();
        waitForStart();
        // Read the bar code with open CV

        long startTime = new Date().getTime();
        long time = 0;

        while (time < 1000 && opModeIsActive()) {
            time = new Date().getTime() - startTime;
            freightLocation = robot.analyze();

            telemetry.addData("Position", freightLocation);
            telemetry.update();
        }

        // Read the bar code with open CV
        robot.strafeLeft(1,.4);
        robot.forward(0.3,1.6);
        robot.turnToHeadingSloppy(.4,-35,0);
        robot.forward(0.3, .4);
        robot.pauseButInSecondsForThePlebeians(.5);
        if (freightLocation == FreightFrenzyComputerVisionRedHub.SkystoneDeterminationPipeline.FreightPosition.LEFT) {
            robot.dumpFreightBottom();
        } else if (freightLocation == FreightFrenzyComputerVisionRedHub.SkystoneDeterminationPipeline.FreightPosition.MIDDLE) {
            robot.dumpFreightMiddle();
        } else if (freightLocation == FreightFrenzyComputerVisionRedHub.SkystoneDeterminationPipeline.FreightPosition.RIGHT) {
            robot.dumpFreightTop();
        }
        robot.turnToHeadingSloppy(.4, -87, 0);
        robot.forward(.5, .4);
        robot.forward(1, -7);

        /*
        robot.strafeRight(0.5,2);
        robot.forward(0.3,0.6);
        //deposit shipping element.

        robot.strafeLeft(0.3,2);
        robot.turnToHeading(0.3,-90,3);
        robot.forward(0.3,4);
        robot.turnToHeading(0.3,0,3);
        robot.backwards(0.3,1);
        robot.forward(0.3,1);
        robot.turnToHeading(0.3,90,3);
        robot.forward(0.5,4.6);
        robot.turnToHeading(0.3,0,3);
        // deposit shipping element

        robot.turnToHeading(0.3,-90,3);
        robot.forward(0.3,4);
        robot.turnToHeading(0.3,0,3);
        robot.backwards(0.3,1);
        robot.forward(0.3,1);
        robot.turnToHeading(0.3,90,3);
        robot.forward(0.5,4.6);
        robot.turnToHeading(0.3,0,3);
*/
    }
}