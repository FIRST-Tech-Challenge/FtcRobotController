package org.firstinspires.ftc.masters;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.Date;

@Autonomous(name = "Pile of red stuff")
public class PileOfRedStuff extends LinearOpMode {
    RobotClass robot;

    @Override
    public void runOpMode() {
        // Here Wayne will do stuff eventualy
        robot = new RobotClass(hardwareMap,telemetry,this);
        robot.openCVInnitShenanigans();
        EasyOpenCVIdentifyShippingElement.SkystoneDeterminationPipeline.FreightPosition freightLocation = null;

        waitForStart();
        // Read the bar code with open CV

        long startTime = new Date().getTime();
        long time = 0;

        while (time < 200 && opModeIsActive()) {
            time = new Date().getTime() - startTime;
            freightLocation = robot.analyze();

            telemetry.addData("Position", freightLocation);
            telemetry.update();
        }

        robot.strafeRight(1,.4);
        robot.forward(0.3,1.6);
        robot.turnToHeadingSloppy(.4,35,0);
        robot.forward(0.3, .4);
        robot.pauseButInSecondsForThePlebeians(.5);

        robot.turnToHeadingSloppy(.4, 87, 0);
        robot.forward(1, -6);


        //deposit shipping element.
        /*
        robot.strafeRight(0.3,2);
        robot.turnToHeading(0.3,90,3);
        robot.forward(0.3,4);
        robot.turnToHeading(0.3,0,3);
        robot.backwards(0.3,1);
        robot.forward(0.3,1);
        robot.turnToHeading(0.3,-90,3);
        robot.forward(0.5,4.6);
        robot.turnToHeading(0.3,0,3);
        // deposit shipping element
        robot.turnToHeading(0.3,90,3);
        robot.forward(0.3,4);
        robot.turnToHeading(0.3,0,3);
        robot.backwards(0.3,1);
        robot.forward(0.3,1);
        robot.turnToHeading(0.3,-90,3);
        robot.forward(0.5,4.6);
        robot.turnToHeading(0.3,0,3);

       */

    }
}
