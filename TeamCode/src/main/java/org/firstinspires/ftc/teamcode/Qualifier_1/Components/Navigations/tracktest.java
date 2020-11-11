package org.firstinspires.ftc.teamcode.Qualifier_1.Components.Navigations;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Qualifier_1.Components.Accesories.Shooter;
import org.firstinspires.ftc.teamcode.Qualifier_1.Robot;


@Autonomous(name= "tracktest")
public class tracktest extends LinearOpMode{


    @Override
    public void runOpMode(){

        Robot robot=new Robot(this);
        Odometry odom = new Odometry();

        telemetry.addData("Status", "Ready to go");
        telemetry.update();

        odom.init(this);
        telemetry.addData("Status", "InitComplete, Ready to Start");
        telemetry.update();
        waitForStart();
        robot.moveForwardOdometry(10,0.5);
        robot.moveForwardOdometry(-10,-0.5);
        robot.moveSideOdometry(10,.5);
        robot.moveSideOdometry(-10,-0.5);
        robot.turnOdometry(90,0.5);
        robot.turnOdometry(-90,-0.5);
        robot.moveAngleOdometry(Math.PI/4,5,5,0.5);
        robot.moveAngleOdometry(-3*Math.PI/4,-5,-5,-0.5);

        stop();
    }



}

