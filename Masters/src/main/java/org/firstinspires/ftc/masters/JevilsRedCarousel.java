package org.firstinspires.ftc.masters;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.Date;

@Autonomous(name = "Red Carousel")
public class JevilsRedCarousel extends LinearOpMode {
    RobotClass robot;

    @Override
    public void runOpMode() {

        robot = new RobotClass(hardwareMap, telemetry, this);
        robot.openCVInnitShenanigans();
        EasyOpenCVIdentifyShippingElement.SkystoneDeterminationPipeline.FreightPosition freightLocation = null;

        waitForStart();

        long startTime = new Date().getTime();
        long time = 0;

        while (time < 200 && opModeIsActive()) {
            time = new Date().getTime() - startTime;
            freightLocation = robot.analyze();

            telemetry.addData("Position", freightLocation);
            telemetry.update();
        }
        robot.forward(0.6, 1.5);
        robot.turnToHeadingSloppy(.6,-45,-20);
        robot.forward(.3,.8);

        //deposit shipping element


//        robot.backwards(0.3,0.2);
//        robot.strafeLeft(0.3,0.2);
//        robot.turnToHeading(0.3,-125, 3);
//        robot.forward(0.3,0.2);
        robot.forward(.3,-2.6);

//        //Jevil turns carousel
        robot.jevilTurnCarousel(.5,2);

        robot.turnToHeadingSloppy(.3,-90,-75);
        robot.strafeRight(.3,.8);
        robot.forward(.3,-0.8);

//        robot.turnToHeading(0.3, -90, 3);
//        robot.strafeRight(0.3,0.2);
//        robot.forward(0.3,0.2);
//        robot.parkRed();
    }
}