package org.firstinspires.ftc.teamcode.Qualifier_1.Autonomous.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Qualifier_1.Robot;

@Autonomous(name = "IMUTest")
public class IMUTest extends LinearOpMode {

    @Override
    public void runOpMode() {

        Robot robot = new Robot(this);
        ElapsedTime runtime = new ElapsedTime();

        waitForStart();

        robot.moveForwardIMU(50, 0.8);
        sleep(1000);
        robot.moveBackwardIMU(50, 0.8);
        sleep(1000);
        robot.moveLeftIMU(70, 0.8, 0, 0.15, 0.2);
        sleep(1000);
        robot.moveRightIMU(70, 0.8, 0, 0.1, 0.2);
        sleep(1000);

    }
}