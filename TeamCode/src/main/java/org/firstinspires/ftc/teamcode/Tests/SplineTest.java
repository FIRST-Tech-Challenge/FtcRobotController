package org.firstinspires.ftc.teamcode.Tests;


import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Components.BasicChassis;
import org.firstinspires.ftc.teamcode.Robot;
//@Disabled

@Autonomous(name= "SplineTest", preselectTeleOp = "OneGPTeleop")
public class SplineTest extends LinearOpMode {
    @Override
    public void runOpMode(){
        Robot robot = new Robot(this, BasicChassis.ChassisType.ENCODER, true, false);
        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        robot.setPosition(0,0,0);
        waitForStart();//robot.goToPosition(72,0,0,0.3);
//        for(int i =0 ; i<4;i++){
//            robot.turnInPlace(76,0.8);
//            robot.turnInPlace(-45,0.8);
//        }
//        amogus
        robot.goToPosition(1, 48,0,90,0.5);
        robot.goToPosition(1, 48,24,200,0.5);
        robot.goToPosition(1, 0,0,90,0.5);
        robot.goToPosition(1, 0,24,-20,0.5);
        robot.goToPosition(1, 48,0,90,0.5);
        robot.goToPosition(1, 48,24,180,0.5);
        robot.goToPosition(1,0,24,-90,0.5);
        robot.goToPosition(1, 0,0,-90,0.5);
        sleep(5000);
//        robot.partOfPolySplineToPositionHead(1,0,0,0,0,30.0/1.5,60,60.0/1.5,0,true,true,0.5);
//        robot.partOfPolySplineToPositionHead(1,0,0,30.0/1.5,60,60.0/1.5,0, 60.0/1.5,-30,true,true,0.5);
//        sleep(3000);
//        robot.partOfPolySplineToPositionHead(0,40,0,40,0,20,20,0,0,true,true,0.5);
//        robot.partOfPolySplineToPositionHead(0,40,0,20,20,0,0,0,-10,true,true,0.5);
//        sleep(3000);
        //barrier pass run demo, start facing forward with left side of chassis alligned with the tile edge one tile from the barrier.
//        robot.goToPosition(1,12,-5,0,0.3);
//        robot.goToPosition(1,2,-5,-97,0.2);
//        robot.goToPosition(1,-0.3,-16,-95,0.2);
//        robot.goToPosition(1,-0.9,-23,-90,0.2);
//        robot.goToPosition(1,-0.9,-32,-90,0.2);
        //spline test
//        robot.partOfPolySplineToPositionHead(1, 0, 0, 0 , 0, 6, 20, 18, 20, true, true, 0.2);
//        sleep(5000);
//        robot.partOfPolySplineToPositionHead(1, 0, 0, 6,20, 18, 20, 28, 26,  true, true, 0.2);
//        //robot.setPosition(20,18,-71);
//        telemetry.addData("14",1);
//        telemetry.update();
//        sleep(5000);
//        robot.partOfPolySplineToPositionHead(1, 6, 20, 18, 20, 28, 26, 28, 40, true, true, 0.2);
//        telemetry.addData("14",1);
//        telemetry.update();
//        sleep(5000);
//        robot.partOfPolySplineToPositionHead(1,  18, 20, 28, 26, 28, 40, 28, 50, true, true, 0.2);
//        telemetry.addData("15",1);
//        telemetry.update();
//        telemetry.addData("AMONGUS",1);
//        telemetry.update();
//        sleep(5000);
        //amongus?
//        robot.partOfPolySplineToPositionHead(1, 0, 0, 0 , 0, 0, 48, -32, 72, true, true, 0.2);
//        robot.partOfPolySplineToPositionHead(1, 0 , 0, 0, 48, -32, 72,-48,48, true, true, 0.2);
//        robot.partOfPolySplineToPositionHead(1, 0, 48, -32, 72,-48,48,-48, 2, true, true, 0.2);
//        robot.partOfPolySplineToPositionHead(1, 32, 72,-48,48,-48, 5,-48,0, true, true, 0.2);
//        robot.partOfPolySplineToPositionHead(0, -48, 0, -48,5,-32,20, 0,0, true, true, 0.2);
//        robot.partOfPolySplineToPositionHead( 0,-48,5,-32,20, 0,0, 0 , -5, true, true, 0.2);
//        robot.goToPosition(0,-24,0,0,0.4);
//        robot.partOfPolySplineToPositionHead(1, 0,-24,0,-24, 3,-1, 24, 2.7, true,true,0.3);
//        robot.partOfPolySplineToPositionHead(1, 0,-24, 3,-1, 24, 2.7, 48,2.7, true,true,0.3);
//        robot.partOfPolySplineToPositionHead(1,  3,-1, 24, 3.0, 48,3.0, 48,2.7, true,true,0.3);
//        telemetry.addData("15",1);
//        telemetry.update();
//        telemetry.addData("AMONGUS",1);
//        telemetry.update();
//        robot.partOfPolySplineToPositionHead(1,  18, 24, 0, 36, -18, 48, 0, 60, false, false, 0.4);
//        robot.partOfPolySplineToPositionHead(1, 0, 48, -18, 48, 0, 60,18, 72, false, false, 0.4);
//        robot.partOfPolySplineToPositionHead(1, -18, 48, 0, 60,18, 72, 0 ,84, false, false, 0.4);
//        robot.partOfPolySplineToPositionHead(1, 0, 60,18, 72, 0 ,84, -18, 72, false, false, 0.4);
//        robot.partOfPolySplineToPositionHead(1, 18, 72, 0 ,84, -18, 72, 0, 60, false, false, 0.4);
//        robot.partOfPolySplineToPositionHead(1, 0 ,84, -18, 72, 0, 60, 18,48, false, false, 0.4);
//        robot.partOfPolySplineToPositionHead(1, -18, 72, 0, 60, 18,48, 0, 36, false, false, 0.4);
//        robot.partOfPolySplineToPositionHead(1, 0, 60, 12,48, 0, 36, -18, 24, false, false, 0.4);
//        robot.partOfPolySplineToPositionHead(1, 18,48, 0, 36, -18, 24, 0,0,false, false, 0.4);
//        robot.partOfPolySplineToPositionHead(1, 0, 36, -18, 24, 0,0, 0, -24,true, true, 0.4);
//        robot.tripleSplineToPositionHead(1, 0, 0, 0, 10, -15, 15, -25, 20, -30, 30, 1.0);
//        robot.goToPosition(48,48,90,1.0);
//        robot.goToPosition(36,36,90*3/4,0.8);
//        robot.goToPosition(24,24,90*1/2,0.6);
//        robot.goToPosition(12,12,90*1/4,0.5);
//        robot.goToPosition(0,0,0,0.4);
        //robot.tripleSplineToPosition(1, 0, 0, 0, 20, 0, 30, 0, 40, 0, 50, 1.0);
        //robot.tripleSplineToPosition(1, 0, 0, 20, 12, 30, 16, 40, 12, 50, 5, 1.0);
        //robot.tripleSplineToPosition(1, 0, 0, 20, 0, 30, 0, 40, 0, 50, 0, 1.0);
        stop();
    }
}

