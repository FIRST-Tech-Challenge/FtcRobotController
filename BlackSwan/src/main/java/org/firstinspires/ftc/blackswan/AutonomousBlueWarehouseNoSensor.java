package org.firstinspires.ftc.blackswan;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
public class AutonomousBlueWarehouseNoSensor extends LinearOpMode{
    Robot robot;
    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap, telemetry, this);
        waitForStart();
        robot.forward(1.5, .5);
        robot.armThing(3);
        robot.right(1.75, .5);
        robot.eject();
        robot.back(.25,.5);
        robot.back(.2, .5);
        robot.turnLeft(70, .5);
        robot.forward(6.5, 1);
        robot.turnLeft(30, .5);
    }
}
