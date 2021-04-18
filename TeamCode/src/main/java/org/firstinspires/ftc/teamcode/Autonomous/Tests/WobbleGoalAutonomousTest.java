/**
 * This is the program for testing the wobble goal arm
 * @author Sid
 * @version 1.0
 * @since 2020-11-01
 */
package org.firstinspires.ftc.teamcode.Autonomous.Tests;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Components.Accesories.WobbleGoal;
import org.firstinspires.ftc.teamcode.Components.BasicChassis;
import org.firstinspires.ftc.teamcode.Robot;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.opMode;

@Autonomous (name = "WobbleGoalAutonomous ", group="Tests: ")

public class WobbleGoalAutonomousTest extends LinearOpMode {


    public void runOpMode(){
        WobbleGoal wobbleGoal = new WobbleGoal(this);
        waitForStart();
        wobbleGoal.goToPosition(WobbleGoal.Position.DROP);
        sleep(1000);
        wobbleGoal.openWobbleGoalClaw();
        sleep(1000);
        wobbleGoal.goToPosition(WobbleGoal.Position.REST);
        sleep(1000);
        wobbleGoal.goToPosition(WobbleGoal.Position.GRAB);
        sleep(1000);
        wobbleGoal.closeWobbleGoalClaw();
        sleep(1000);
        wobbleGoal.goToPosition(WobbleGoal.Position.AutoRAISE);
        sleep(1000);
        wobbleGoal.goToPosition(WobbleGoal.Position.DROP);
        sleep(1000);
        wobbleGoal.openWobbleGoalClaw();
        sleep(1000);
        wobbleGoal.goToPosition(WobbleGoal.Position.REST);
        sleep(1000);

    }
}
