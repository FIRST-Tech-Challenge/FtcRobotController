/**
 * This is the autonomous program for Position 3.
 *
 * The robot shoots 3 rings into the middle goal, delivers the wobble goal to the Position 3 square,
 * and parks on the Launch Line.
 *
 * @author Sai
 * @version 1.0
 * @since 11/26/2020
 * @status finished
 */
package org.firstinspires.ftc.teamcode.Qualifier_1.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Qualifier_1.Components.BasicChassis;
import org.firstinspires.ftc.teamcode.Qualifier_1.Robot;

@Autonomous(name = "R_mid3_P3w_park_1")
@Disabled
public class R_mid3_P3w_park extends LinearOpMode {

    @Override
    public void runOpMode() {
        Robot robot=new Robot(this, BasicChassis.ChassisType.IMU);
        ElapsedTime runtime = new ElapsedTime();

        waitForStart();
        robot.moveWobbleGoalServo(false);
        //Move to Shooting Position
        robot.moveAngle(-13, -57, 0.5);
        telemetry.addData("Moving Backwards", 60);
        telemetry.update();
        //Shoot 2 rings into the Middle Goal
        robot.shootHighGoal(3);
        telemetry.addData("Shooting Rings", 3);
        telemetry.update();
        //Adjust the Wobble Goal's Position
        robot.moveAngle(3, 0, 0.5);
        //Go to Position 3
        robot.moveAngle(-30.5, -34, 0.5);
        telemetry.addData("MoveAngle", 3);
        telemetry.update();
        robot.moveBackward(7, 0.5);
        //Release the Wobble Goal and Park on the Launch Line
        robot.moveWobbleGoalServo(true);
        sleep(1000);
        robot.moveAngle(8, 40, 0.5);
    }
}