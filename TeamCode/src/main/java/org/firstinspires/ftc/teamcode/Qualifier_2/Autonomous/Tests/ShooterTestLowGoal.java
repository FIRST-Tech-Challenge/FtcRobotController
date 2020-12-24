package org.firstinspires.ftc.teamcode.Qualifier_2.Autonomous.Tests;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Qualifier_2.Components.Accesories.Shooter;


/**
 * Shooter Testing for low goal
 *
 * @author  Nikhil
 * @version 1.0
 * @since   2020-October-26
 *
 */
@Autonomous(name= "Shooter Test Low Goal ", group="Tests: ")
@Disabled
public class ShooterTestLowGoal extends LinearOpMode{

    @Override
    public void runOpMode(){
        telemetry.addData("Status", "Ready to go");
        telemetry.update();
        Shooter shooter = null;
        shooter = new Shooter(this);

        telemetry.addData("Status", "InitComplete, Ready to Start");
        telemetry.update();
        shooter.shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        waitForStart();
        shooter.shootLowGoal(200000);

    }



}
