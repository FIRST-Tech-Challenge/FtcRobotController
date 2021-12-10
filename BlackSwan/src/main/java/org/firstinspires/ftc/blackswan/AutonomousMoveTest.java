package org.firstinspires.ftc.blackSwan;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.blackswan.Robot;

@Autonomous

public class AutonomousMoveTest extends LinearOpMode {
    Robot robot;
    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap,telemetry,this);
        waitForStart();
        robot.left(3,.5);
        robot.pause(5000);
        robot.right(3,.5);
        robot.pause(5000);

    }
}
