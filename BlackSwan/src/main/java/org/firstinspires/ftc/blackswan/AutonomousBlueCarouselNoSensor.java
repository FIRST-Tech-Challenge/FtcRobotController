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
        robot.liftForMovement();
        // turns to rotate carousel and rotates carousel
        robot.right(.5,.5);
        robot.turnRight(45,.5);
        robot.right(.5,.5);
        robot.back(.47,.5);
        robot.redCarousel(4000);
        //check the code underneath, wrong places, too jerky
        robot.forward(.47,.5);
        robot.left(.5,.5);
        robot.turnRight(30,.5);
        robot.left(3,.5);
        robot.armThing(3);
        robot.forward(.9,.5); //adjust to make it work
        robot.eject();
        robot.back(.8,.5); //match the forward
        robot.turnRight(.20,.5);
        robot.right(4.25,.5);
        robot.forward(.8,.5); // adjust to make it work

    }
}
