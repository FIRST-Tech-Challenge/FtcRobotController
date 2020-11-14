package org.firstinspires.ftc.teamcode.Qualifier_1.Autonomous.Tests;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Qualifier_1.Components.Accesories.Shooter;
import org.firstinspires.ftc.teamcode.Qualifier_1.Components.Navigations.Odometry;
import org.firstinspires.ftc.teamcode.Qualifier_1.Robot;


@Autonomous(name= "tracktest")
public class tracktest extends LinearOpMode{
final boolean debug = true;

    @Override
    public void runOpMode(){

        Robot robot=new Robot(this);
        Odometry odom = new Odometry();

        telemetry.addData("Status", "Ready to go");
        telemetry.update();

        odom.init(this);
        telemetry.addData("Status", "InitComplete, Ready to Start");
        telemetry.update();
        sleep(1000);
        waitForStart();
        robot.moveAngleOdometry(0,5,0.5);
        robot.moveAngleOdometry(0,-5,0.5);
        robot.moveAngleOdometry( 0,5,0.5);
        robot.moveAngleOdometry( 0,-5,0.5);
        robot.moveAngleOdometry( 5,5,0.5);
        robot.moveAngleOdometry( -5,-5,0.5);
        sleep(500);
        stop();
    }



}

