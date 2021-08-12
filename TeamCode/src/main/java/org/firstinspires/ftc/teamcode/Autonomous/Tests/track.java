package org.firstinspires.ftc.teamcode.Autonomous.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Components.OdometryChassis;

@Autonomous(name= "track")
public class track extends LinearOpMode {
    final boolean debug= true;
    @Override
    public void runOpMode(){
        //Robot robot = new Robot(this, BasicChassis.ChassisType.ODOMETRY, false, false);
        ElapsedTime op = new ElapsedTime();
        OdometryChassis odom = new OdometryChassis(this, false,true);

        //ElapsedTime runtime = new ElapsedTime();
        odom.setPosition(0,0,-36);
        //int rings = robot.getRingsAndWaitForStart();
        //robot.stopRingDetection();
        waitForStart();
//        robot.setPosition(61.75,-42.25, 0);
//        robot.moveAngle(0,-65,0.5);
//        sleep(5000);
//        robot.goToPosition(23.5,-23.5,0,0.5);
        //robot.navigate();
        while(!isStopRequested()){
            odom.track();
        }
        stop();
    }



}
