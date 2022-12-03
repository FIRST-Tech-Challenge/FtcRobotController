package org.firstinspires.ftc.blackswan;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous

public class RedBlueAuto extends LinearOpMode {
    Robot robot;

    @Override
    public void runOpMode() {
        robot = new Robot(hardwareMap,telemetry,this);
        waitForStart();
        robot.forward(1,.5);
        robot.left(1,.5);
        robot.right(1,.5);
        robot.turnLeft(30, 1);
        robot.turnRight(60, 1);
        robot.back(1,.5);
    }
}