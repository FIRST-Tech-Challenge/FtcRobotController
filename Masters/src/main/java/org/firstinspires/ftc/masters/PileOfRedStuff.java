package org.firstinspires.ftc.masters;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.Date;

@Autonomous(name = "warehouse red", group="competition")
public class PileOfRedStuff extends LinearOpMode {
    RobotClass robot;

    @Override
    public void runOpMode() {
        robot = new RobotClass(hardwareMap,telemetry,this);
        robot.openCVInnitShenanigans();
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

       // robot.strafeRight(1,.4);
        robot.forward(0.3,1.6);
        if (freightLocation== FreightFrenzyComputerVisionRedHub.SkystoneDeterminationPipeline.FreightPosition.LEFT|| freightLocation == FreightFrenzyComputerVisionRedHub.SkystoneDeterminationPipeline.FreightPosition.MIDDLE){
            robot.forward(0.4, 1.6);
            robot.forward(0.3, -1.6);
        }
        robot.turnToHeadingSloppy(.4,35,0);


        if (freightLocation== FreightFrenzyComputerVisionRedHub.SkystoneDeterminationPipeline.FreightPosition.LEFT ){
            robot.forward(0.3, 1.15);
        }else if (freightLocation == FreightFrenzyComputerVisionRedHub.SkystoneDeterminationPipeline.FreightPosition.MIDDLE){
            robot.forward(0.3, 1.05);
        } else if (freightLocation == FreightFrenzyComputerVisionRedHub.SkystoneDeterminationPipeline.FreightPosition.RIGHT){
            robot.forward(0.3, 0.9);
        }
        robot.pauseButInSecondsForThePlebeians(.5);
        if (freightLocation == FreightFrenzyComputerVisionRedHub.SkystoneDeterminationPipeline.FreightPosition.LEFT) {
            robot.dumpFreightBottom();
        } else if (freightLocation == FreightFrenzyComputerVisionRedHub.SkystoneDeterminationPipeline.FreightPosition.MIDDLE) {
            robot.dumpFreightMiddle();
        } else if (freightLocation == FreightFrenzyComputerVisionRedHub.SkystoneDeterminationPipeline.FreightPosition.RIGHT) {
            robot.dumpFreightTop();
        }
        robot.forward(0.3, -0.4);
        robot.turnToHeadingSloppy(.4, 87, 0);
        robot.forward(1, -6);




    }
}
