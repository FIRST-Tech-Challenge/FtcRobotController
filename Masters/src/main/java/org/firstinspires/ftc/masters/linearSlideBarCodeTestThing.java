package org.firstinspires.ftc.masters;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.Date;

//For this to work, the robot needs to start on the red carousel side.

@Autonomous(name = "Linear Slide and bar code test thing.")
public class linearSlideBarCodeTestThing extends LinearOpMode {
    RobotClass robot;

    @Override
    public void runOpMode() {

        robot = new RobotClass(hardwareMap, telemetry, this);

        //This stuff is the open CV, used when we get the placing the freight done

//        robot.openCVInnitShenanigans();
//        EasyOpenCVIdentifyShippingElement.SkystoneDeterminationPipeline.FreightPosition freightLocation = null;
//
//        waitForStart();
//
//        long startTime = new Date().getTime();
//        long time = 0;
//
//        while (time < 200 && opModeIsActive()) {
//            time = new Date().getTime() - startTime;
//            freightLocation = robot.analyze();
//
//            telemetry.addData("Position", freightLocation);
//            telemetry.update();
//        }
//        robot.forward(0.4, .8);
//        robot.turnToHeadingSloppy(.25,-45,-45);

//        if (freightLocation == EasyOpenCVIdentifyShippingElement.SkystoneDeterminationPipeline.FreightPosition.LEFT) {



//        } else if (freightLocation == EasyOpenCVIdentifyShippingElement.SkystoneDeterminationPipeline.FreightPosition.MIDDLE) {

//        } else if (freightLocation == EasyOpenCVIdentifyShippingElement.SkystoneDeterminationPipeline.FreightPosition.RIGHT) {

//        }


    }
}
