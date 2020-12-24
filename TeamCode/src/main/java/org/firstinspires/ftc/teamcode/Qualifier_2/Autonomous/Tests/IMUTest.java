package org.firstinspires.ftc.teamcode.Qualifier_2.Autonomous.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Qualifier_2.Components.BasicChassis;
import org.firstinspires.ftc.teamcode.Qualifier_2.Robot;

@Disabled
@Autonomous(name = "IMUTest ", group="Tests: ")
public class IMUTest extends LinearOpMode {
    @Override

    public void runOpMode() {

        Robot robot=new Robot(this, BasicChassis.ChassisType.IMU);
        ElapsedTime runtime = new ElapsedTime();

        waitForStart();

        robot.moveLeft(69, 0.75);
        sleep(1000);
        robot.turnInPlace(0, 0.5);
//        robot.moveBackward(48, 0.5);
//        sleep(1000);
//        robot.turnInPlace(0, 0.5);
//        robot.moveLeft(24, 0.5);
//        sleep(1000);
//        robot.turnInPlace(0, 0.5);
        robot.moveRight(69, 0.75);
        sleep(1000);
        robot.turnInPlace(0, 0.5);

    }
}