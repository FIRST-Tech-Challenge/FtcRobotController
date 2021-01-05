package org.firstinspires.ftc.teamcode.Autonomous.Tests;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Components.Accesories.Shooter;
import org.firstinspires.ftc.teamcode.Components.BasicChassis;
import org.firstinspires.ftc.teamcode.Robot;


/**
 * Shooter Testing for mid goal
 *
 * @author  Nikhil
 * @version 1.0
 * @since   2020-October-26
 *
 */
@Autonomous(name= "Shooter Test Mid Goal ", group="Tests: ")
@Disabled
public class ShooterTestMidGoal extends LinearOpMode{
    //private Shooter shooter=null;

    @Override
    public void runOpMode(){
        telemetry.addData("Status", "Ready to go");
        telemetry.update();
        Robot robot = new Robot(this, BasicChassis.ChassisType.ODOMETRY, false, false);
        telemetry.addData("Status", "InitComplete, Ready to Start");
        telemetry.update();
        waitForStart();
        robot.shootMiddleGoal(3);

    }



}