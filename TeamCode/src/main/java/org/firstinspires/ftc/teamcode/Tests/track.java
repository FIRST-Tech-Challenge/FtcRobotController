package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Components.EncoderChassis;
import org.firstinspires.ftc.teamcode.Components.VSLAMChassis;

@Autonomous(name= "track")
public class track extends LinearOpMode {
    final boolean debug= true;
    @Override
    public void runOpMode(){
        //Robot robot = new Robot(this, BasicChassis.ChassisType.ODOMETRY, false, false);
        ElapsedTime op = new ElapsedTime();
        EncoderChassis odom = new EncoderChassis(this, false,false);

        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        //ElapsedTime runtime = new ElapsedTime();
        odom.setPosition(0,0,0);
        //int rings = robot.getRingsAndWaitForStart();
        //robot.stopRingDetection();
        waitForStart();
        odom.moveTester();
//        robot.setPosition(61.75,-42.25, 0);
//        robot.moveAngle(0,-65,0.5);
//        sleep(5000);
//        robot.goToPosition(23.5,-23.5,0,0.5);            robot.partOfPolySplineToPositionHead(1,  -6.75,-2.2,robot.track()[0],robot.track()[1],2.75, 3.1, 25.75,3.15,true,true,0.3);
//            robot.partOfPolySplineToPositionHead(1,  -6.75,-2.2, robot.track()[0], robot.track()[1], 25.75,3.15, 30.75,3.0,true,true,0.3);
        //robot.navigate();
        while(!isStopRequested()){
            odom.track();
        }
        stop();
    }



}
