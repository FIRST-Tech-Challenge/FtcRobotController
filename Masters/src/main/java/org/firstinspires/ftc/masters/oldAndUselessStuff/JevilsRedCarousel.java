package org.firstinspires.ftc.masters;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Red Carousel")
public class JevilsRedCarousel extends LinearOpMode {
    RobotClass robot;

    @Override
    public void runOpMode() {
        waitForStart();
        // Read the bar code with open CV
        robot.strafeRight(0.3,0.2);
        robot.forward(0.3,0.2);
        //deposit shipping element
        robot.backwards(0.3,0.2);
        robot.strafeLeft(0.3,0.2);
        robot.turnToHeading(0.3,-125, 3);
        robot.forward(0.3,0.2);
        //Jevil turns carousel
        robot.turnToHeading(0.3, -90, 3);
        robot.strafeRight(0.3,0.2);
        robot.forward(0.3,0.2);
        robot.parkRed();
    }
}