package org.firstinspires.ftc.teamcode.Qualifier_1.Autonomous.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.Qualifier_1.Components.Accesories.WobbleGoal;

@Autonomous (name = "WobbleGoalAutonomous")
public class WobbleGoalAutonomousTest extends LinearOpMode {

    public void runOpMode(){
        DcMotor motor = (DcMotor) hardwareMap.dcMotor.get ("wobbleGoalMotor");
        WobbleGoal wobbleGoal;
        wobbleGoal = new WobbleGoal(this);

        //wobbleGoal.liftingPosition();

    }
}
