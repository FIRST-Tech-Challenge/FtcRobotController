package org.firstinspires.ftc.blackSwan;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous

public class AutonomousRedCarousel extends LinearOpMode {
    Robot robot;
    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap,telemetry,this);
        waitForStart();
        robot.left(.5,.5);
        robot.turnLeft(45,.5);
        robot.left(.5,.5);
        robot.back(.47,.5);
        robot.redCarousel(3000);
        robot.forward(.47,.5);
        robot.right(.5,.5);
        robot.turnLeft(25,.5);
        robot.pause(10);
        robot.forward(.5,.5);
        robot.pause(10);
        robot.right(.5,.5);
        robot.pause(10);
        robot.forward(.35,.5);
        robot.pause(10);
        robot.turnLeft(.15,.5);
        robot.pause(100); //replace with color sensor check
        robot.right(.75,.5);
        robot.pause(100); //replace with color sensor check
        robot.back(.5,.5);
        robot.turnRight(45,.5);
        robot.forward(.75,.5);
        robot.pause(8000);
        //do the arm thingy here
        robot.back(.75,.5);
        robot.turnLeft(45,.5);
        robot.left(2.5,.75);
        robot.forward(1.5,.5);




    }
}
