package org.firstinspires.ftc.teamcode.Qualifier_1.Autonomous.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Qualifier_1.Robot;

@Autonomous(name = "newChassis2Test")
public class NewTest_Sai extends LinearOpMode {

    Robot robot = new Robot();
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        robot.initChassis(this);
        waitForStart();
        //Tests all of the motors individually
        sleep(1000);
        robot.moveMotorLeftFront(10);
        telemetry.addData("Moving LeftFront Motor", 10);
        telemetry.update();
        sleep(1000);
        robot.moveMotorRightFront(10);
        telemetry.addData("Moving RightFront Motor", 10);
        telemetry.update();
        sleep(1000);
        robot.moveMotorLeftBack(10);
        telemetry.addData("Moving LeftBack Motor", 10);
        telemetry.update();
        sleep(1000);
        robot.moveMotorRightBack(1000);
        telemetry.addData("Moving RightBack Motor", 10);
        telemetry.update();
        sleep(1000);

        //Tests the motors moving together
        robot.moveForward(25, 0.75);
        telemetry.addData("Moving Forward", 10);
        telemetry.update();
        sleep(1000);
        robot.moveBackward(25, 0.75);
        telemetry.addData("Moving Backward", 10);
        telemetry.update();
        sleep(1000);
        robot.moveLeft(25, 0.75);
        telemetry.addData("Moving Left", 10);
        telemetry.update();
        sleep(1000);
        robot.moveRight(25, 0.75);
        telemetry.addData("Moving Right", 10);
        telemetry.update();
        sleep(1000);

        //Moving with IMU
        robot.moveForwardIMU(50,0.8);
        telemetry.addData("Moving Forward IMU", 50);
        telemetry.update();
        sleep(1000);
        robot.moveBackwardIMU(50,0.8);
        telemetry.addData("Moving Backward IMU", 10);
        telemetry.update();
        sleep(3000);
        robot.moveLeftIMU(70, 0.8, 0, 0.15, 0.2);
        telemetry.addData("Moving Left IMU", 0);
        telemetry.update();
        sleep(1000);
        robot.moveRightIMU(70, 0.8, 0, 0.1, 0.2);
        telemetry.addData("Moving Right IMU", 0);
        telemetry.update();
        sleep(1000);

        //Tests Moving at a Specific Angle (I am testing how the moveAngle2 function works, I am still not completely sure what it is)
        robot.moveAngle2(30, 45, 0.5);
        telemetry.addData("Moving Angle", 0.5);
        telemetry.update();
        sleep(1000);
        robot.moveAngle2(30, 45, 0.8);
        telemetry.addData("Moving Angle", 0.8);
        telemetry.update();
        sleep(1000);

    }
}