package org.firstinspires.ftc.teamcode.Autonomous;


import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Components.Accesories.WobbleGoal;
import org.firstinspires.ftc.teamcode.Components.BasicChassis;
import org.firstinspires.ftc.teamcode.Components.OdometryChassis;
import org.firstinspires.ftc.teamcode.Robot;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

@Autonomous(name= "SplineTest", preselectTeleOp = "OneGPTeleop")
public class SplineTest extends LinearOpMode {
    @Override
    public void runOpMode(){
        Robot robot = new Robot(this, BasicChassis.ChassisType.ODOMETRY, true, false);
        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        waitForStart();
//        robot.goToPosition(72,0,0,1.0);
        robot.partOfPolySplineToPositionHead(1, 0, 0, 18, 24, 0, 36, -18, 48, true, false, 0.4);
        robot.partOfPolySplineToPositionHead(1, 0, 0, 18, 24, 0, 36, -18, 48, false, false, 0.4);
        robot.partOfPolySplineToPositionHead(1,  18, 24, 0, 36, -18, 48, 0, 60, false, false, 0.4);
        robot.partOfPolySplineToPositionHead(1, 0, 48, -18, 48, 0, 60,18, 72, false, false, 0.4);
        robot.partOfPolySplineToPositionHead(1, -18, 48, 0, 60,18, 72, 0 ,84, false, false, 0.4);
        robot.partOfPolySplineToPositionHead(1, 0, 60,18, 72, 0 ,84, -18, 72, false, false, 0.4);
        robot.partOfPolySplineToPositionHead(1, 18, 72, 0 ,84, -18, 72, 0, 60, false, false, 0.4);
        robot.partOfPolySplineToPositionHead(1, 0 ,84, -18, 72, 0, 60, 18,48, false, false, 0.4);
        robot.partOfPolySplineToPositionHead(1, -18, 72, 0, 60, 18,48, 0, 36, false, false, 0.4);
        robot.partOfPolySplineToPositionHead(1, 0, 60, 12,48, 0, 36, -18, 24, false, false, 0.4);
        robot.partOfPolySplineToPositionHead(1, 18,48, 0, 36, -18, 24, 0,0,false, false, 0.4);
        robot.partOfPolySplineToPositionHead(1, 0, 36, -18, 24, 0,0, 0, -24,true, true, 0.4);
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

