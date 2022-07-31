package org.firstinspires.ftc.blackswan;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous

public class AutonomousRedCarouselV2 extends LinearOpMode {
    Robot robot;
    int cubethingwithlevels = 0;

    @Override
    public void runOpMode() {
        robot = new Robot(hardwareMap,telemetry,this);
        waitForStart();
        robot.liftForMovement();
        // turns to rotate carousel and rotates carousel
        robot.left(.5,.5);
        robot.turnLeft(45,.5);
        robot.left(.5,.5);
        robot.back(.47,.5);
        robot.redCarousel(4000);
        robot.forward(.47,.5);
        robot.right(.5,.5);
        robot.turnLeft(30,.5);
        //scoring stuff
        //this original bit moved right 3 so everything should add up to 0 up and down change and right 3
        robot.right(3,.5);
        //arm stuff
        robot.armThing(3);
        robot.forward(.9,.5);
        robot.dumpRight();
        robot.back(.8,.5);
        robot.turnLeft(.20,.5);
        robot.left(4.25,.5);
        robot.forward(.8,.5); // adjust to make it work

    }
}
