package org.firstinspires.ftc.blackswan;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous

public class AutonomousRedCarouselNoSensor extends LinearOpMode {
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
        //check the code underneath, wrong places, too jerky
        robot.forward(.47,.5);
        robot.right(.5,.5);
        robot.turnLeft(30,.5);
        robot.right(3,.5);
        robot.armThing(3);
        robot.forward(.8,.5); //adjust to make it work
        robot.eject();
        robot.back(.75,.5); //match the forward
        robot.turnLeft(.20,.5);
        robot.left(4.25,.5);
        robot.forward(.8,.5); // adjust to make it work

    }
}
