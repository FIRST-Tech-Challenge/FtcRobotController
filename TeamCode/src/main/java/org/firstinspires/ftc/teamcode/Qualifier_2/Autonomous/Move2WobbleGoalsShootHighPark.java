/**
 * Moves 2 wobble goals, shoots 3 high goals, and parks
 * @author  Aamod
 * @volgate 13.6-13.8 V
 * USE BATTERY 4 and 5 (Batter 5 works better)
 */

package org.firstinspires.ftc.teamcode.Qualifier_2.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Qualifier_2.Components.Accesories.WobbleGoal;
import org.firstinspires.ftc.teamcode.Qualifier_2.Components.BasicChassis;
import org.firstinspires.ftc.teamcode.Qualifier_2.Robot;

@Autonomous(name = "Move2WobbleGoalsShootHighPark")
public class Move2WobbleGoalsShootHighPark extends LinearOpMode {
    @Override
    public void runOpMode() {

        Robot robot = new Robot(this, BasicChassis.ChassisType.IMU);
        ElapsedTime runtime = new ElapsedTime();

        int rings = robot.runTensorFlowWaitForStart();

//        waitForStart();
//        rings = robot.tensorFlow.getNumberOfRings();
        robot.stopTensorFlow();

        if (rings == 4) {
            robot.moveBackward(53, 0.8);
            sleep(50);
            robot.turnInPlace(3, 0.6);
            sleep(200);
            robot.shootHighGoal(3);
            sleep(100);

            robot.turnInPlace(-19, 0.6);
            sleep(200);
            robot.moveBackward(30, 0.85);
            sleep(150);
            robot.moveBackward(18, 0.65);
            sleep(100);
            robot.moveBackward(5, 0.5);
            sleep(100);
            robot.turnInPlace(-23, 0.6);
            sleep(200);
            robot.wobbleGoalGoToPosition(WobbleGoal.Position.GRAB);
            robot.moveForward(76, 0.85);
            sleep(150);
            robot.turnInPlace(-87, 0.65);
            sleep(200);
            robot.moveLeft(10, 0.7);
            sleep(150);
            robot.wobbleGoalGoToPosition(WobbleGoal.Position.RAISE);
            sleep(750);
            robot.turnInPlace(-7, 0.5);
            sleep(200);
            robot.moveBackward(80, 0.85);
            sleep(100);
            robot.turnInPlace(35, 0.6);
            sleep(100);
            robot.wobbleGoalGoToPosition(WobbleGoal.Position.DROP);
            robot.moveRight(5, 0.5);
            robot.turnInPlace(0, 0.6);
            robot.wobbleGoalGoToPosition(WobbleGoal.Position.REST);
            robot.moveForward(35, 0.875);
//            telemetry.addData("NumberOfRings: ", 4);
//            telemetry.update();
//            sleep(2000);
            stop();
        } else if (rings == 1) {
            robot.moveBackward(53, 0.8);
            sleep(50);
            robot.turnInPlace(3, 0.6);
            sleep(200);
            robot.shootHighGoal(3);
            sleep(100);

            robot.turnInPlace(6, 0.5);
            sleep(100);
            robot.moveBackward(25, 0.85);
            sleep(75);
            robot.moveWobbleGoalServo(true);
            robot.moveForward(8, 0.85);
            sleep(75);
            robot.turnInPlace(-13,0.6);
            sleep(100);
            robot.wobbleGoalGoToPosition(WobbleGoal.Position.GRAB);
            sleep(100);
            robot.moveForward(35, 0.85);
            sleep(75);
            robot.turnInPlace(-87, 0.6);
            sleep(200);
            robot.moveLeft(10, 0.6);
            sleep(200);
            robot.wobbleGoalGoToPosition(WobbleGoal.Position.RAISE);
            sleep(250);
            robot.turnInPlace(0, 0.6);
            sleep(150);
            robot.moveAngle(21,-72, 0.6);
            sleep(75);
            robot.turnInPlace(0, 0.8);
            sleep(100);
            robot.moveLeft(3, 0.8);
            sleep(200);
            robot.wobbleGoalGoToPosition(WobbleGoal.Position.DROP);
            sleep(200);
            robot.moveRight(9, 0.85);
            robot.wobbleGoalGoToPosition(WobbleGoal.Position.REST);
            sleep(75);
            robot.moveForward(26, 0.85);
//            telemetry.addData("NumberOfRings: ", 1);
//            telemetry.update();
//            sleep(2000);
            stop();
        } else {
            robot.moveBackward(53, 0.8);
            sleep(50);
            robot.turnInPlace(3, 0.6);
            sleep(200);
            robot.shootHighGoal(3);
            sleep(100);

            robot.turnInPlace(-60, 0.5);
            sleep(200);
            robot.moveBackward(12, 0.4);
            sleep(100);
            robot.wobbleGoalGoToPosition(WobbleGoal.Position.GRAB);
            sleep(100);
            robot.moveForward(41, 0.8);
            sleep(200);
            robot.moveLeft(16, 0.8);
            sleep(500);
            robot.wobbleGoalGoToPosition(WobbleGoal.Position.RAISE);
            sleep(500);
            robot.moveBackward(27, 0.55);
            sleep(100);
            robot.turnInPlace(70, 0.3);
            sleep(200);
            robot.moveLeft(5, 0.6);
            sleep(200);
            robot.wobbleGoalGoToPosition(WobbleGoal.Position.DROP);
            sleep(2000);
            robot.moveRight(12, 0.7);
            sleep(100);
            robot.wobbleGoalGoToPosition(WobbleGoal.Position.REST);
            sleep(200);
            robot.turnInPlace(23, 0.5);
            sleep(100);
            robot.moveBackward(30, 0.8);
//            telemetry.addData("NumberOfRings: ", 0);
//            telemetry.update();
//            sleep(2000);
            stop();
        }
    }
}
