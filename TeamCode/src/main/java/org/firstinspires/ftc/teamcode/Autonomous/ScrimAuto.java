package org.firstinspires.ftc.teamcode.Autonomous;


import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Components.BasicChassis;
import org.firstinspires.ftc.teamcode.Robot;

@Autonomous(name= "ScrimAuto", preselectTeleOp = "OneGPTeleop")
public class ScrimAuto extends LinearOpMode {
    @Override
    public void runOpMode(){
        Robot robot = new Robot(this, BasicChassis.ChassisType.VSLAM, false, false);
        //int position = robot.BlueElemTest(this,0,0);
        double[] turretTarget = {0,0,0};//{hubx-position*3/2,huby-position*3/2,1+7*position}
        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        robot.setPosition(0,0,0);
        waitForStart();
//        robot.TurretSlidesToPosition(-11.5,10.0,0,0.5);
        robot.goToPosition(0,-28,-10,-15,0.5);
//        robot.FlipBasketArmToPosition(.5);
//        sleep(1000);
//        robot.FlipBasketToPosition(0.01);
//        sleep(1000);
//        robot.FlipBasketToPosition(0.5);
//        sleep(500);
//        robot.TurretSlidesToPosition(0,0,0,0.5);
        robot.goToPosition(1,-9.4,-27.2,-30,0.4);
//        robot.FlipBasketArmToPosition(0.00);
//        robot.FlipBasketToPosition(1.0);
        robot.spinCarouselAutonomousBlue();
        /** 3, -49; 19.5, -62.3; 36, -31; **/
        robot.partOfPolySplineToPositionHead(0,-25.5,-5.6,-25.5,-5.6,2,-52,20,-62,true,true,0.5);
        robot.partOfPolySplineToPositionHead(0,-25.6,-5.6, 2,-52,27,-64,36,-32,true,true,0.5);
        robot.partOfPolySplineToPositionHead(0, 2,-52,27,-64,36,-32,36,0,true,true,0.5);
        robot.turnInPlace(90,0.5);
        sleep(1000);
        if(!robot.goToPositionTeleop(1, 52, -29, 1.0)) {
            stop();
        }
        robot.partOfPolySplineToPositionHead(1,40,-27, 81,-27,99,-21,99,-12,true,true,0.5);
        robot.partOfPolySplineToPositionHead(1, 81,-27,99,-21,99,-16, 99,0,true,true,0.5);
        if(!robot.autoIntake(0.2,20)){
            stop();
        }
        sleep(2000);
        stop();
    }
}


