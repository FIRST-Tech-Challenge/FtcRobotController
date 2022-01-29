package org.firstinspires.ftc.blackswan;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous

public class AutonomousBlueCarouselNoSensor extends LinearOpMode {
    Robot robot;
    int cubethingwithlevels = 0;

    @Override
    public void runOpMode() {
        robot = new Robot(hardwareMap,telemetry,this);
        waitForStart();
        //setup for fieldcentric driving
        robot.forward(1,.5);
        robot.turnLeft(90,.5);
        robot.left(1.25,.5);
        //movelift
        robot.liftForMovement();
        // turns to rotate carousel and rotates carousel
        //robot.left(.5,.5);
        robot.turnRight(45,.5);
        robot.right(.5,.5);
        robot.back(.55,.5);
        robot.redCarousel(4000);
        robot.forward(.47,.5);
        robot.left(.5,.5);
        robot.turnRight(40,.5);
        //scoring stuff
        robot.left(3,.5);
        robot.forward(.5,.5); //adjust to make it work
        robot.armThing(3);
        robot.forward(0.4,0.5);
        robot.eject();
        robot.back(.1,.5); //match the forward
        robot.right(4.5,.5);
        robot.armThing(0);
        robot.forward(0.5,0.2);

    }
}
