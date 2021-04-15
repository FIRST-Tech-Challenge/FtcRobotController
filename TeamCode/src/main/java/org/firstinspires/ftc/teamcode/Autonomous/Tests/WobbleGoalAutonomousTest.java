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

@Autonomous (name = "WobbleGoalAutonomous ", group="Tests: ")

public class WobbleGoalAutonomousTest extends LinearOpMode {

    public void runOpMode(){

        waitForStart();
        DcMotor wobbleGoalMotor;
        wobbleGoalMotor = (DcMotorEx) hardwareMap.dcMotor.get("wobbleGoalMotor");
        wobbleGoalMotor.setPower(0.05);
        wobbleGoalMotor.setTargetPosition(280);
        wobbleGoalMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sleep(5000);
        wobbleGoalMotor.setTargetPosition(90);
        wobbleGoalMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sleep(5000);
    }
}
