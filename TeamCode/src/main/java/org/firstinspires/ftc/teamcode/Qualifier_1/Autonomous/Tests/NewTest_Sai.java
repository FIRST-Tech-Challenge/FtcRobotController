/**
 * This test program tests the four chassis motors.
 * I am still troubleshooting the ShooterMotor and the wobbleGoalMotor
 *
 * Everyone should run this code when you arrive at the garage to make sure the robot is working properly.
 *
 * Everyone should also run it before leaving to make sure the robot is not broken.
 *
 *
 * @author Sai
 * @version 1.0
 * @since 11/14/2020
 * @status work in progress
 */

package org.firstinspires.ftc.teamcode.Qualifier_1.Autonomous.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Qualifier_1.Robot;

@Autonomous(name = "NewTest_Sai")
//@Disabled
public class NewTest_Sai extends LinearOpMode {

    @Override
    public void runOpMode() {

        ElapsedTime runtime = new ElapsedTime();
        Robot robot = new Robot(this);

        waitForStart();
        //Tests all of the motors individually
        telemetry.addData("So Far It Is Working", 0);
        telemetry.update();
        sleep(3000);
        robot.moveMotorLeftFront(10);
        telemetry.addData("Moving LeftFront Motor", 10);
        telemetry.update();
        sleep(1000);
        robot.moveMotorLeftFront(-10);
        telemetry.addData("Moving LeftFront Motor", -10);
        telemetry.update();
        sleep(1000);
        robot.moveMotorRightFront(10);
        telemetry.addData("Moving RightFront Motor", 10);
        telemetry.update();
        sleep(1000);
        robot.moveMotorRightFront(-10);
        telemetry.addData("Moving RightFront Motor", -10);
        telemetry.update();
        sleep(1000);
        robot.moveMotorLeftBack(10);
        telemetry.addData("Moving LeftBack Motor", 10);
        telemetry.update();
        sleep(1000);
        robot.moveMotorLeftBack(-10);
        telemetry.addData("Moving LeftBack Motor", -10);
        telemetry.update();
        sleep(1000);
        robot.moveMotorRightBack(10);
        telemetry.addData("Moving RightBack Motor", 10);
        telemetry.update();
        sleep(1000);
        robot.moveMotorRightBack(-10);
        telemetry.addData("Moving RightBack Motor", -10);
        telemetry.update();
        sleep(1000);

//        robot.moveShooterMotor(1, 1);
//        telemetry.addData("Moving ShooterMotor", 1);
//        telemetry.update();
//        sleep(1000);
//        robot.moveShooterMotor(-0.5);
//        telemetry.addData("Moving ShooterMotor", -10);
//        telemetry.update();
//        sleep(1000);
//
//        robot.moveWobbleGoalMotor(0.1);
//        telemetry.addData("Moving wobbleGoalMotor", 10);
//        telemetry.update();
//        sleep(1000);
//        robot.moveWobbleGoalMotor(-0.1);
//        telemetry.addData("Moving wobbleGoalMotor", -10);
//        telemetry.update();
//        sleep(1000);
//
//        //Tests Servo
//        robot.setShooterServoPosition(-0.1);

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
    }
}