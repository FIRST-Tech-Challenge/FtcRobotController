/*
 * This is a silly testing program for making sure ethe
 * robot will self distruct after running for 5 sec.
 *
 * Everyone should run it when you arrive in the garage
 * to make sure the robot is working.
 *
 * Everyone should run it before you leavet he garage
 * to make sure you did not break the robot.
 *
 * @author  Sai
 * @version 1.0
 * @since   2020-11-02
 */

package org.firstinspires.ftc.teamcode.Qualifier_1.Autonomous.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Qualifier_1.Robot;

@Autonomous(name = "NewTest_Sai")
public class NewTest_Sai extends LinearOpMode {

    Robot robot = new Robot();
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        robot.initChassis(this);
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

        robot.moveShooterMotor(10);
        telemetry.addData("Moving ShooterMotor", 10);
        telemetry.update();
        sleep(1000);
        robot.moveShooterMotor(-10);
        telemetry.addData("Moving ShooterMotor", -10);
        telemetry.update();
        sleep(1000);

        robot.moveWobbleGoalMotor(10);
        telemetry.addData("Moving wobbleGoalMotor", 10);
        telemetry.update();
        sleep(1000);
        robot.moveWobbleGoalMotor(-10);
        telemetry.addData("Moving wobbleGoalMotor", -10);
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