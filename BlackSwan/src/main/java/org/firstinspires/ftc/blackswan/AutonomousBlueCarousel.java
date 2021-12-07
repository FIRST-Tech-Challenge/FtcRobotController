package org.firstinspires.ftc.blackswan;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.blackswan.Robot;
@Autonomous
public class AutonomousBlueCarousel  extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot;
        robot = new Robot(hardwareMap, telemetry, this);
        waitForStart();
        robot.right(.5, .5);
        robot.turnRight(45, .5);
        robot.right(.5, .5);
        robot.back(.47, .5);
        robot.redCarousel(3000);
        robot.forward(.47, .5);
        robot.left(.5, .5);
        robot.turnRight(25, .5);
        robot.pause(10);
        robot.forward(.5, .5);
        robot.pause(10);
        robot.left(.5, .5);
        robot.pause(10);
        robot.forward(.35, .5);
        robot.pause(10);
        robot.turnRight(.15, .5);
        robot.pause(100); //replace with color sensor check
        robot.left(.75, .5);
        robot.pause(100); //replace with color sensor check
        robot.back(.5, .5);
        robot.turnLeft(45, .5);
        robot.forward(.75, .5);
        robot.armThing(3);
        robot.back(.75, .5);
        robot.turnRight(45, .5);
        robot.left(2.5, .75);
        robot.forward(1.5, .5);

    }
}
