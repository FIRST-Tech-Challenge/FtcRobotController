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

        final int rings = robot.getRingsAndWaitForStart();

//        waitForStart();
//        rings = robot.tensorFlow.getNumberOfRings();
        robot.stopRingDetection();
        sleep(100);
        robot.moveWobbleGoalToPosition(WobbleGoal.Position.RUN);
//        robot.shootGoalTeleop(120);
        robot.navigate();
        if(rings==1 || rings==4) {
            robot.moveBackward(24, 0.8);
            robot.turnInPlace(-5, 1);
            robot.shootHighGoal(3);
            sleep(200);
            robot.turnInPlace(0, 1);
            robot.moveAngle(-7, -6, 1);
            robot.moveBackward(27, 1);
            robot.moveRight(18, 1);
            robot.startIntake();
            robot.startTransfer();
            robot.moveForward(8, 0.75);
            sleep(100);
            robot.shootHighGoal(1);
            if(rings==4) {
                robot.moveForward(10, 1);
                robot.shootHighGoal(3);
                sleep(200);
                robot.stopIntake();
                robot.stopTransfer();
            }
        }
        if(!(rings==1 || rings==4)) {
            robot.moveAngle(-5, -65, 0.8);
            robot.openWobbleGoalClaw();
            sleep(250);
            robot.moveAngle(41, 7, 0.8);
            robot.shootThreePowerShot();
            robot.moveWobbleGoalToPosition(WobbleGoal.Position.RAISE);
            robot.turnInPlace(0, 1.0);
            robot.moveAngle(1, 49, 0.8);
            robot.moveAngle(-3,0,0.8);
            robot.closeWobbleGoalClaw();
            sleep(250);
            robot.moveWobbleGoalToPosition(WobbleGoal.Position.RUN);
            robot.moveAngle(-29,-59,0.7);
            robot.moveAngle(6,0,0.7);
        }
    }
}
