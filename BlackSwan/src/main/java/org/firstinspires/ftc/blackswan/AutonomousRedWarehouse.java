package org.firstinspires.ftc.blackSwan;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.blackSwan.Robot;

@Autonomous

public class AutonomousRedWarehouse extends LinearOpMode {
    Robot robot;
    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap,telemetry,this);
        waitForStart();
        robot.forward(1.5,.5);
        robot.left(2,.5);
        // this is where the dropping code will go
        robot.back(.2,.5);
        robot.turnRight(70,.5);
        robot.forward(5,1);
        robot.turnRight(30,.5);
    }
}
