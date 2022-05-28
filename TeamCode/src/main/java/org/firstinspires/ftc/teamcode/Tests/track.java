package org.firstinspires.ftc.teamcode.Tests;

import static org.firstinspires.ftc.teamcode.Components.EncoderChassis.aVelocity;
import static java.lang.Math.abs;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Components.EncoderChassis;

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
        double stopStart = 1;
//        odom.moveTester();
//        robot.setPosition(61.75,-42.25, 0);
//        robot.moveAngle(0,-65,0.5);
//        sleep(5000);
//        robot.goToPosition(23.5,-23.5,0,0.5);            robot.partOfPolySplineToPositionHead(1,  -6.75,-2.2,robot.track()[0],robot.track()[1],2.75, 3.1, 25.75,3.15,true,true,0.3);
//            robot.partOfPolySplineToPositionHead(1,  -6.75,-2.2, robot.track()[0], robot.track()[1], 25.75,3.15, 30.75,3.0,true,true,0.3);
        //robot.navigate();
        double acceleRate = 0;
        sleep(1000);
        odom.track();
        odom.turnInPlace(50,1.0);
        odom.turnInPlace(-90,1.0);
        odom.turnInPlace(160,1.0);
        odom.turnInPlace(0,1.0);
        //846.125,0.47, 833, 0.43
        odom.stopAllMotors();
        while(abs(aVelocity)>1){
            odom.track();
        }
        telemetry.addData("deceleTime", getRuntime()-1);
        telemetry.addData("acceleRate", acceleRate);
        telemetry.update();
        sleep(10000);
        stop();
    }



}
