package org.firstinspires.ftc.blackswan;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
public class AutonomousRedWarehouseNoSensor extends LinearOpMode{
    Robot robot;
    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap, telemetry, this);
        waitForStart();
        robot.liftForMovement();
        robot.forward(1.5, .5);
        robot.armThing(3);
        robot.left(1.75, .5);
        robot.eject();
        robot.back(.25,.5);
        robot.back(.2, .5);
        robot.turnRight(70, .5);
        robot.forward(6.5, 1);
        robot.turnRight(30, .5);
    }
}
