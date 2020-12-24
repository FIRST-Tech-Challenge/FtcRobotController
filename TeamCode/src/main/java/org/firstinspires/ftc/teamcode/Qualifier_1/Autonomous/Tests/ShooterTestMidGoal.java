package org.firstinspires.ftc.teamcode.Qualifier_1.Autonomous.Tests;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Qualifier_1.Components.Accesories.Shooter;


/**
 * Shooter Testing for mid goal
 *
 * @author  Nikhil
 * @version 1.0
 * @since   2020-October-26
 *
 */
@Autonomous(name= "Shooter Test Mid Goal_1")
@Disabled
public class ShooterTestMidGoal extends LinearOpMode{
    //private Shooter shooter=null;

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
        shooter.shootMidGoal(200000);

    }



}