/**
 * Tests Tensor Flow. If Tensor Flow detects
 * 4 rings, the robot should move forward.
 * If it detects 1 ring, the robot should
 * move left. Otherwise, the robot should move
 * right.
 *
 * @author  Aamod
 * @version 1.0
 * @since   2021-January-17
 * @status: Not fully working
 */

package org.firstinspires.ftc.teamcode.Autonomous.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Components.Accesories.WobbleGoal;
import org.firstinspires.ftc.teamcode.Components.BasicChassis;
import org.firstinspires.ftc.teamcode.Components.ObjectDetection.TensorFlow;
import org.firstinspires.ftc.teamcode.Robot;

import java.util.ArrayList;

@Autonomous(name= "TensorFlowTest2 ", group="Tests: ")

public class TensorFlowTest2 extends LinearOpMode {

    @Override
    public void runOpMode() {

        Robot robot = new Robot(this, BasicChassis.ChassisType.IMU, true, false);
        ElapsedTime runtime = new ElapsedTime();

        int rings = robot.getRingsAndWaitForStart();

        robot.stopRingDetection();

        robot.moveBackward(53, 0.8);
        sleep(200);
        robot.turnInPlace(-3, 0.6);
        sleep(500);
        robot.shootHighGoal(3);
        sleep(200);

        if (rings == 4) {
            robot.turnInPlace(-13, 0.6);
            sleep(200);
            robot.moveBackward(30, 0.8);
            sleep(50);
            robot.moveBackward(18, 0.65);
            sleep(50);
            robot.moveBackward(5, 0.5);
            sleep(200);
            robot.moveForward(5, 0.8);
            sleep(200);
            robot.turnInPlace(7, 0.6);
            sleep(200);
            robot.moveForward(27, 0.8);
            sleep(2000);

//            robot.moveBackward(6, 0.5);
//            sleep(200);
//            robot.turnInPlace(-6, 0.6);
//            sleep(200);
//            robot.moveBackward(70, 0.8);
//            sleep(200);
//            robot.moveBackward(28, 0.5);
//            sleep(200);
//            robot.moveWobbleGoalServo(true);
//            sleep(500);
//            robot.turnInPlace(8, 0.6);
//            robot.moveForward(37, 0.8);
            telemetry.addData("NumberOfRings: ", 4);
            telemetry.update();
            sleep(2000);
        } else if (rings == 1) {
            robot.turnInPlace(10, 0.5);
            sleep(200);
            robot.moveBackward(25, 0.8);
            sleep(200);
            robot.moveForward(8, 0.8);

//            robot.moveBackward(6, 0.5);
//            sleep(200);
//            robot.turnInPlace(4,0.8);
//            sleep(200);
//            robot.moveBackward(80,0.8);
//            sleep(200);
//            robot.moveWobbleGoalServo(true);
//            sleep(1000);
//            robot.moveForward(8,0.8);
            telemetry.addData("NumberOfRings: ", 1);
            telemetry.update();
            sleep(2000);
        } else {
            robot.turnInPlace(-60, 0.5);
            sleep(200);
            robot.moveBackward(12, 0.6);
            sleep(100);
            robot.moveWobbleGoalToPosition(WobbleGoal.Position.GRAB);
            sleep(100);
            robot.moveForward(42, 0.8);
            sleep(100);
            robot.moveLeft(15, 0.8);
            sleep(500);
            robot.moveWobbleGoalToPosition(WobbleGoal.Position.RAISE);
            sleep(500);
            robot.moveBackward(33, 0.6);
            sleep(100);
            robot.turnInPlace(75, 0.7);
            sleep(200);
            robot.moveWobbleGoalToPosition(WobbleGoal.Position.DROP);
            sleep(450);
            robot.moveRight(8, 0.7);
            sleep(100);
            robot.moveWobbleGoalToPosition(WobbleGoal.Position.REST);
            sleep(200);
            robot.turnInPlace(23, 0.5);
            sleep(100);
            robot.moveBackward(30, 0.8);
//            robot.moveBackward(6, 0.5);
//            sleep(200);
//            robot.turnInPlace(-20, 0.6);
//            sleep(200);
//            robot.moveForward(6, 0.8);
//            robot.moveWobbleGoalServo(true);
//            sleep(1000);
            telemetry.addData("NumberOfRings: ", 0);
            telemetry.update();
            sleep(2000);
            stop();
        }
    }
}
