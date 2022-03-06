package org.firstinspires.ftc.teamcode.Autonomous;


import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Components.BasicChassis;
import org.firstinspires.ftc.teamcode.Robot;
//cock
@Autonomous(name= "BlueLeftP", preselectTeleOp = "OneGPTeleop")
public class BlueLeftP extends LinearOpMode {
    @Override
    public void runOpMode(){
        Robot robot = new Robot(this, BasicChassis.ChassisType.VSLAM, false, false);

        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        robot.setPosition(0,0,0);
        waitForStart();
//        robot.TurretSlidesToPosition(-11.5,10.0,0,0.5);
        robot.goToPosition(0,-10,2,15,0.5);
//        robot.FlipBasketArmToPosition(.5);
//        sleep(1000);
//        robot.FlipBasketToPosition(0.01);
//        sleep(1000);
//        robot.FlipBasketToPosition(0.5);
//        sleep(500);
//        robot.TurretSlidesToPosition(0,0,0,0.5);
        robot.partOfPolySplineToPositionHead(1,2,-10,2,-10,-1,-2, -25,0,true,true,0.5);
        robot.partOfPolySplineToPositionHead(1,2,-10,-1,-2, -25,0,-50,0,true,true,0.5);

        stop();
    }
}