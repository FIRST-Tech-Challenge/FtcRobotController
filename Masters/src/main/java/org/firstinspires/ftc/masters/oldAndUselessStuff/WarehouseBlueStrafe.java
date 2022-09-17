package org.firstinspires.ftc.masters.oldAndUselessStuff;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.masters.FreightFrenzyComputerVisionRedHub;
import org.firstinspires.ftc.masters.RobotClass;

import java.util.Date;

@Disabled
//@Autonomous(name = "warehouse blue strafe", group="competition")
public class WarehouseBlueStrafe extends LinearOpMode {
    RobotClass robot;

    @Override
    public void runOpMode() {
        // Here Wayne will do stuff eventualy
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

        robot.strafeLeft(0.3,2.2);
        robot.forward(0.3,1.5);
//        if (freightLocation== EasyOpenCVIdentifyShippingElement.SkystoneDeterminationPipeline.FreightPosition.LEFT|| freightLocation == EasyOpenCVIdentifyShippingElement.SkystoneDeterminationPipeline.FreightPosition.MIDDLE){
//            robot.forward(0.4, 1.6);
//            robot.forward(0.3, -1.6);
//        }
//        robot.turnToHeadingSloppy(.4,35,0);


        if (freightLocation== FreightFrenzyComputerVisionRedHub.SkystoneDeterminationPipeline.FreightPosition.LEFT ){
            robot.forward(0.3, .2);
        }
//        else if (freightLocation == EasyOpenCVIdentifyShippingElement.SkystoneDeterminationPipeline.FreightPosition.MIDDLE){
//            robot.forward(0.3, 1.05);
//        } else if (freightLocation == EasyOpenCVIdentifyShippingElement.SkystoneDeterminationPipeline.FreightPosition.RIGHT){
//            robot.forward(0.3, 1);
//        }
        robot.pauseButInSecondsForThePlebeians(.5);
        if (freightLocation == FreightFrenzyComputerVisionRedHub.SkystoneDeterminationPipeline.FreightPosition.LEFT) {
            robot.dumpFreightBottom();
            robot.forward(0.3, -0.2);
        } else if (freightLocation == FreightFrenzyComputerVisionRedHub.SkystoneDeterminationPipeline.FreightPosition.MIDDLE) {
            robot.dumpFreightMiddle();
        } else if (freightLocation == FreightFrenzyComputerVisionRedHub.SkystoneDeterminationPipeline.FreightPosition.RIGHT) {
            robot.dumpFreightTop();
        }

        robot.turnToHeadingSloppy(.4, -90, 15);
        robot.forward(0.4, -1);
        robot.strafeRight(0.3,0.6);
        robot.forward(1, -7);




    }
}
