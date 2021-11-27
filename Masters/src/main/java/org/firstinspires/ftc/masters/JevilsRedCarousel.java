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

        // Here Wayne will do stuff eventually
        waitForStart();

        long startTime = new Date().getTime();
        long time = 0;

        while (time < 200 && opModeIsActive()) {
            time = new Date().getTime() - startTime;
            freightLocation = robot.analyze();

            telemetry.addData("Position", freightLocation);
            telemetry.update();
        }
        robot.forward(0.4, .8);
        robot.turnToHeadingSloppy(.25,-45,-45);
        //deposit shipping element
        robot.mecanumWitchcraftBackLeft(.4,1.3);
//        robot.backwards(0.3,0.2);
//        robot.strafeLeft(0.3,0.2);
//        robot.turnToHeading(0.3,-125, 3);
//        robot.forward(0.3,0.2);
//        //Jovil turns carousel
//        robot.turnToHeading(0.3, -90, 3);
//        robot.strafeRight(0.3,0.2);
//        robot.forward(0.3,0.2);
//        robot.parkRed();
    }
}