package org.firstinspires.ftc.blackswan;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.blackswan.Robot;

@Autonomous

public class AutonomousBlueWarehouse extends LinearOpMode {
    Robot robot;
    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap,telemetry,this);
        waitForStart();
        robot.forward(1.5,.5);
        robot.right(2,.5);
        //robot.armThing(3,.5);
        //robot.drop();
        robot.back(.2,.5);
        robot.turnLeft(70,.5);
        robot.forward(5,1);
        robot.turnLeft(30,.5);
    }
}
