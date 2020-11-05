package org.firstinspires.ftc.teamcode.Qualifier_1.Accesories;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.Qualifier_1.Components.Accesories.WobbleGoal;

@Autonomous (name = "WobbleGoalAutonomous")
public class WobbleGoalAutonomousTest extends LinearOpMode {

    public void runOpMode(){
        DcMotorEx motor = (DcMotorEx) hardwareMap.dcMotor.get ("wobbleGoalMotor");
        WobbleGoal wobbleGoal = new WobbleGoal(motor, this);

        telemetry.addData("Robot status: ", "about to call wait for start");
        telemetry.update();
        waitForStart();

        telemetry.addData("Robot status: ", "about to move motor");
        telemetry.update();




        wobbleGoal.droppingPosition();

        telemetry.addData("Robot status: ", "moved motor");
        telemetry.update();

    }
}
