/**
 * This is the autonomous program for Position 2.
 *
 * The robot shoots 3 rings into the middle goal, delivers the wobble goal to the Position 2 square,
 * and parks on the Launch Line.
 *
 * @author Sai
 * @version 1.0
 * @since 11/26/2020
 * @status finished
 */

package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Components.BasicChassis;
import org.firstinspires.ftc.teamcode.Robot;

@Disabled
@Autonomous(name = "R_mid3_P2w_park")
public class R_mid3_P2w_park extends LinearOpMode {

    @Override
    public void runOpMode() {

        Robot robot=new Robot(this, BasicChassis.ChassisType.IMU, false, false);
        ElapsedTime runtime = new ElapsedTime();

        waitForStart();
        robot.openWobbleGoalClaw();
        robot.moveBackward(59, 0.5);
        robot.shootHighGoal(3);
        robot.moveBackward(20, 0.5);
        sleep(1500);
        robot.closeWobbleGoalClaw();
        robot.moveForward(10, 0.5);
    }
}