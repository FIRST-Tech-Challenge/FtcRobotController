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
        robot.right(1,.5);
        //movelift
        robot.liftForMovement();
        // turns to rotate carousel and rotates carousel
        robot.right(.5,.5);
        robot.turnRight(45,.5);
        robot.right(.5,.5);
        robot.back(.47,.5);
        robot.blueCarousel(4000);
        //check the code underneath, wrong places, too jerky
        robot.forward(.47,.5);
        robot.left(.5,.5);
        robot.turnRight(30,.5);
        robot.left(3.4,.5);
        robot.armThing(3);
        robot.forward(.95,.5); //adjust to make it work
        robot.eject();
        robot.back(.85,.5);
        robot.turnRight(.20,.5);
        robot.right(4.4,.5);
        robot.forward(1.3,.5); // adjust to make it work

    }
}
