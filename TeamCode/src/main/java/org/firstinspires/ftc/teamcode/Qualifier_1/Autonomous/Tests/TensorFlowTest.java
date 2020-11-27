/**
 * Tests Tensor Flow. If Tensor Flow detects
 * 4 rings, the robot should move forward.
 * If it detects 1 ring, the robot should
 * move left. Otherwise, the robot should move
 * right.
 *
 * @author  Aamod
 * @version 1.0
 * @since   2020-November-5
 * @status: Not fully working
 */

package org.firstinspires.ftc.teamcode.Qualifier_1.Autonomous.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Qualifier_1.Components.Accesories.WobbleGoal;
import org.firstinspires.ftc.teamcode.Qualifier_1.Components.BasicChassis;
import org.firstinspires.ftc.teamcode.Qualifier_1.Components.Chassis;
import org.firstinspires.ftc.teamcode.Qualifier_1.Robot;

import java.awt.font.NumericShaper;
import java.util.ArrayList;

@Autonomous
//@Disabled
public class TensorFlowTest extends LinearOpMode {

    @Override
    public void runOpMode() {

        Robot robot = new Robot(this, BasicChassis.ChassisType.IMU);
        ElapsedTime runtime = new ElapsedTime();

        int rings = -1;
        int i = 0;

        int numOfTime4Rings = 0;
        int numOfTime1Ring = 0;
        int numOfTime0Rings = 0;

        int arraySize = 11;
        ArrayList<Integer> NumberOfRings = new ArrayList<Integer>(arraySize);

        robot.initTensorFlow();

        for (int index = 0; index<arraySize; index++) {
            robot.runTensorFlow();
            sleep(10);
            rings = robot.tensorFlow.getNumberOfRings();
            NumberOfRings.add(index, rings);
            //telemetry.addData("11 Number of Rings: ", "i=%4d %d", i++, rings);
            //telemetry.update();
        }
        //sleep(2000);

//        telemetry.addData("Number of Rings: ", "%d %d %d %d %d %d", NumberOfRings.get(0),NumberOfRings.get(1),NumberOfRings.get(2),NumberOfRings.get(3),NumberOfRings.get(4), NumberOfRings.get(5));
//        telemetry.addData("Number of Rings: ", "%d %d %d %d %d", NumberOfRings.get(6),NumberOfRings.get(7),NumberOfRings.get(8),NumberOfRings.get(9),NumberOfRings.get(10));
//        telemetry.update();
//        sleep(2000);

        while(!opModeIsActive() && !isStopRequested()) {
            numOfTime4Rings = 0;
            numOfTime1Ring = 0;
            numOfTime0Rings = 0;

            robot.runTensorFlow();
            NumberOfRings.remove(0);
            NumberOfRings.add(robot.tensorFlow.getNumberOfRings());
            telemetry.addData("Number of Rings: ", "%d %d %d %d %d %d", NumberOfRings.get(0),NumberOfRings.get(1),NumberOfRings.get(2),NumberOfRings.get(3),NumberOfRings.get(4), NumberOfRings.get(5));
            telemetry.addData("Number of Rings: ", "%d %d %d %d %d", NumberOfRings.get(6),NumberOfRings.get(7),NumberOfRings.get(8),NumberOfRings.get(9),NumberOfRings.get(10));

            for (i = 0; i<arraySize; i++) {
                if (NumberOfRings.get(i) == 4) {
                    numOfTime4Rings++;
                } else if (NumberOfRings.get(i) == 1) {
                    numOfTime1Ring++;
                } else {
                    numOfTime0Rings++;
                }
            }

            telemetry.addData("Rings Summary: ", "4-rings: %2d 1-ring: %2d 0-rings: %2d", numOfTime4Rings, numOfTime1Ring, numOfTime0Rings);

            if (numOfTime4Rings>numOfTime1Ring && numOfTime4Rings>=numOfTime0Rings){
                rings = 4;
            } else if (numOfTime1Ring>numOfTime4Rings && numOfTime1Ring>=numOfTime0Rings) {
                rings = 1;
            } else {
                rings = 0;
            }

            telemetry.addData("FinalNumOfRings: ", rings);
            telemetry.update();
            sleep(100);
        }

//        waitForStart();
//        rings = robot.tensorFlow.getNumberOfRings();
        robot.stopTensorFlow();

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
            robot.wobbleGoalGoToPosition(WobbleGoal.Position.GRAB);
            sleep(100);
            robot.moveForward(42, 0.8);
            sleep(100);
            robot.moveLeft(15, 0.8);
            sleep(500);
            robot.wobbleGoalGoToPosition(WobbleGoal.Position.RAISE);
            sleep(500);
            robot.moveBackward(33, 0.6);
            sleep(100);
            robot.turnInPlace(75, 0.7);
            sleep(200);
            robot.wobbleGoalGoToPosition(WobbleGoal.Position.DROP);
            sleep(450);
            robot.moveRight(8, 0.7);
            sleep(100);
            robot.wobbleGoalGoToPosition(WobbleGoal.Position.REST);
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
