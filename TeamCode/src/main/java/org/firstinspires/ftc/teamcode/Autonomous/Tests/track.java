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
        Robot robot = new Robot(this, BasicChassis.ChassisType.ODOMETRY, true, true);
        ElapsedTime op = new ElapsedTime();
        OdometryChassis odom= new OdometryChassis(this);

        //ElapsedTime runtime = new ElapsedTime();

        //int rings = robot.getRingsAndWaitForStart();
        //robot.stopRingDetection();
        waitForStart();
        op.startTime();
        robot.setPosition(-15.75,61.75, 0);
        sleep(5000);
        robot.goToPosition(0,0,0,0.8);
        stop();
    }



}
