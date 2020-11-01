package org.firstinspires.ftc.teamcode.Qualifier_1.Accesories;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.Qualifier_1.Components.Accesories.WobbleGoal;

@Autonomous (name = "WobbleGoalAutonomous")
public class WobbleGoalAutonomousTest extends LinearOpMode {

    public void runOpMode(){

        DcMotorEx motor = (DcMotorEx) hardwareMap.dcMotor.get ("wobbleGoalMotor");
        WobbleGoal wobbleGoal = new WobbleGoal(motor);


        wobbleGoal.position1();

    }
}
