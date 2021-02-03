/**
 * Moves 2 wobble goals, shoots 3 high goals, and parks
 * @author  Aamod
 * @volgate 13.6-13.8 V
 * USE BATTERY 4 and 5 (Batter 5 works better)
 */

package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Components.Accesories.WobbleGoal;
import org.firstinspires.ftc.teamcode.Components.BasicChassis;
import org.firstinspires.ftc.teamcode.Robot;

@Autonomous(name = "WobbleGoal2ShootAllHighPark")
public class WobbleGoal2ShootAllHighPark extends LinearOpMode {
    @Override
    public void runOpMode() {

        Robot robot = new Robot(this, BasicChassis.ChassisType.ODOMETRY, true, false);
        ElapsedTime runtime = new ElapsedTime();

        int rings = robot.getRingsAndWaitForStart();

//        waitForStart();
//        rings = robot.tensorFlow.getNumberOfRings();
        robot.stopRingDetection();
        robot.moveWobbleGoalToPosition(WobbleGoal.Position.RAISE);
        robot.navigate();
        robot.moveBackward(25,0.75);
        robot.turnInPlace(-5,0.8);
        robot.shootHighGoal(3);
        sleep(200);
        robot.turnInPlace(0,0.8);
        robot.moveAngle(-10,-4, 0.7);
        robot.moveBackward(28, 0.8);
        robot.moveRight(16, 0.8);
        robot.startIntake();
        sleep(100);
        robot.startTransfer();
        sleep(100);
        robot.moveForward(10, 0.8);
        robot.moveBackward(3, 0.9);
        robot.turnInPlace(5, 0.8);
        robot.moveForward(10, 0.8);
        robot.moveBackward(10, 0.8);
        robot.turnInPlace(-5, 0.8);
        robot.moveForward(10, 0.8);
        robot.moveBackward(10, 0.8);
        robot.shootHighGoal(3);
        sleep(200);


        if (rings == 0) {
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
            robot.moveWobbleGoalToPosition(WobbleGoal.Position.GRAB);
            robot.moveForward(76, 0.85);
            sleep(150);
            robot.turnInPlace(-87, 0.65);
            sleep(200);
            robot.moveLeft(10, 0.7);
            sleep(150);
            robot.moveWobbleGoalToPosition(WobbleGoal.Position.RAISE);
            sleep(750);
            robot.turnInPlace(-7, 0.5);
            sleep(200);
            robot.moveBackward(80, 0.85);
            sleep(100);
            robot.turnInPlace(35, 0.6);
            sleep(100);
            robot.moveWobbleGoalToPosition(WobbleGoal.Position.DROP);
            robot.moveRight(5, 0.5);
            robot.turnInPlace(0, 0.6);
            robot.moveWobbleGoalToPosition(WobbleGoal.Position.REST);
            robot.moveForward(35, 0.875);
//            telemetry.addData("NumberOfRings: ", 4);
//            telemetry.update();
//            sleep(2000);
            stop();
        } else if (rings == 4) {
            robot.moveAngle(10,-36, 0.8);
//            telemetry.addData("NumberOfRings: ", 1);
//            telemetry.update();
//            sleep(2000);
            stop();
        } else {
            robot.moveAngle(20,20,0.75);
//            telemetry.addData("NumberOfRings: ", 0);
//            telemetry.update();
//            sleep(2000);
        }
        robot.moveWobbleGoalClaw(false);
    }
}
