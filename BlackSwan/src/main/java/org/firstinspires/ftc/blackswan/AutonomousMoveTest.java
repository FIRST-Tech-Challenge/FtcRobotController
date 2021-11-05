package org.firstinspires.ftc.blackswan;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Autonomous

public class AutonomousMoveTest extends LinearOpMode {
    Robot robot;
    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap,telemetry,this);
        waitForStart();
        robot.forward(1,.5);
        robot.turnRight(90, .1);
        robot.left(1,.5);
        robot.right(1,.5);
        robot.turnLeft(90,.1);
        robot.backward(1,.5);
    }
}
