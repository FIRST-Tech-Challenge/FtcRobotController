package org.firstinspires.ftc.teamcode.Autonomous.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Components.Accesories.WobbleGoal;
import org.firstinspires.ftc.teamcode.Components.BasicChassis;
import org.firstinspires.ftc.teamcode.Components.OdometryChassis;
import org.firstinspires.ftc.teamcode.Robot;

@Autonomous(name= "track")
public class track extends LinearOpMode {
    final boolean debug= true;
    @Override
    public void runOpMode(){
        Robot robot = new Robot(this, BasicChassis.ChassisType.ODOMETRY, false, false);
        ElapsedTime op = new ElapsedTime();

        //ElapsedTime runtime = new ElapsedTime();

        //int rings = robot.getRingsAndWaitForStart();
        //robot.stopRingDetection();
        waitForStart();
        while(!isStopRequested()){

        }
        stop();
    }



}
