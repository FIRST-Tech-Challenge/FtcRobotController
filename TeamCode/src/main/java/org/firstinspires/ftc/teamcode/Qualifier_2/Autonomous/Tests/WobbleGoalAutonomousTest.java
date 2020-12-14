/**
 * This is the program for testing the wobble goal arm
 * @author Sid
 * @version 1.0
 * @since 2020-11-01
 */
package org.firstinspires.ftc.teamcode.Qualifier_2.Autonomous.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Qualifier_2.Components.Accesories.WobbleGoal;

@Autonomous (name = "WobbleGoalAutonomous")
@Disabled
public class WobbleGoalAutonomousTest extends LinearOpMode {

    public void runOpMode(){
        DcMotor motor = (DcMotor) hardwareMap.dcMotor.get ("wobbleGoalMotor");
        WobbleGoal wobbleGoal;
        wobbleGoal = new WobbleGoal(this);

        //wobbleGoal.liftingPosition();

    }
}
