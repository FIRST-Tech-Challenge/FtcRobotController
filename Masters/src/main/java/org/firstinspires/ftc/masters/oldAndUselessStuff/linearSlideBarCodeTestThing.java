package org.firstinspires.ftc.masters.oldAndUselessStuff;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.masters.FreightFrenzyComputerVisionRedHub;
import org.firstinspires.ftc.masters.RobotClass;

import java.util.Date;

//For this to work, the robot needs to start on the red carousel side.

@Disabled
//@Autonomous(name = "Linear Slide and bar code test thing.", group ="test")
public class linearSlideBarCodeTestThing extends LinearOpMode {
    RobotClass robot;

    @Override
    public void runOpMode() {

        robot = new RobotClass(hardwareMap, telemetry, this);

        //This stuff is the open CV, used when we get the placing the freight done

        robot = new RobotClass(hardwareMap, telemetry, this);
        robot.openCVInnitShenanigans("red");
        FreightFrenzyComputerVisionRedHub.SkystoneDeterminationPipeline.FreightPosition freightLocation = null;

        waitForStart();

        long startTime = new Date().getTime();
        long time = 0;

        while (time < 200 && opModeIsActive()) {
            time = new Date().getTime() - startTime;
            freightLocation = robot.analyze();

            telemetry.addData("Position", freightLocation);
            telemetry.update();
        }
        robot.forward(0.6, 1.6);
        robot.turnToHeadingSloppy(.6,-45,-20);
        robot.forward(.3,.8);

        if (freightLocation == FreightFrenzyComputerVisionRedHub.SkystoneDeterminationPipeline.FreightPosition.LEFT) {
            robot.dumpFreightBottom();
        } else if (freightLocation == FreightFrenzyComputerVisionRedHub.SkystoneDeterminationPipeline.FreightPosition.MIDDLE) {
            robot.dumpFreightMiddle();
        } else if (freightLocation == FreightFrenzyComputerVisionRedHub.SkystoneDeterminationPipeline.FreightPosition.RIGHT) {
            robot.dumpFreightTop();
        }
    }
}
