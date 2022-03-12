package org.firstinspires.ftc.teamcode.Autonomous;


import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Components.BasicChassis;
import org.firstinspires.ftc.teamcode.Robot;

@Autonomous(name= "RedRightPWarehouse", preselectTeleOp = "OneGPTeleop")
public class RedRightPWarehouse extends LinearOpMode {
    @Override
    public void runOpMode(){
        Robot robot = new Robot(this, BasicChassis.ChassisType.VSLAM, false, false);
        sleep(1000);
        int position = robot.BlueElemTest(this,0,0);
        double[] turretTarget = {12+10.6,-24+16.2,0};//{hubx-position*3/2,huby-position*3/2,1+7*position}
        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        double moved = 12;
        robot.setPosition(0,0,0);
        waitForStart();
        robot.TurretSlidesToPosition(0,0,0,0.4);
        if(position!=2) {
            robot.goToPosition(0, -14, 10-moved, 45, 0.5);
            if(position==1) {
                robot.TurretSlidesToPosition(0, 12.242,0, 0.5);
                sleep(1300);
                robot.FlipBasketArmToPosition(.6);
                sleep(500);
                robot.FlipBasketToPosition(0.0);
                sleep(1000);
            }
            if(position==0) {
                robot.TurretSlidesToPosition(0, 8.442, 0, 0.5);
                sleep(1300);
                robot.FlipBasketArmToPosition(.75);
                sleep(500);
                robot.FlipBasketToPosition(0.0);
                sleep(1000);
            }
            robot.FlipBasketToPosition(0.4);
            robot.TurretSlidesToPosition(0, 0, 0, 0.5);
            robot.FlipBasketArmToPosition(0.00);
            robot.goToPosition(1, -8, 33-moved, 40, 0.4);
            robot.setMotorPowers(0.2);
            robot.spinCarouselAutonomousRed();
            robot.setMotorPowers(-0.3);
            sleep(500);
            robot.goToPosition(0, -6, 0-moved, 80, 0.4);
            robot.partOfPolySplineToPositionHead(0,33-moved,-8,0-moved,-6,-40-moved,-3.3,-69-moved,-3.3,true,true,0.5);
            robot.partOfPolySplineToPositionHead(0,0-moved,-6,-40-moved,-3.3,-69-moved,-3.3, -72-moved,-8,true,true,0.5);
//        robot.partOfPolySplineToPositionHead(0,-25.5,-5.6,-25.5,-5.6,12,-50,16,-38,true,true,0.5);
//        robot.partOfPolySplineToPositionHead(0,-25.5,-5.6,12,-50,35,-27,16,-38,true,true,0.5);
//        robot.turnInPlace(90,0.5);
//        robot.goToPosition(1,-25,81,90,1.0);
//        robot.partOfPolySplineToPositionHead(1,40,-27, 81,-27,99,-21,99,-12,true,true,0.5);
//        robot.partOfPolySplineToPositionHead(1, 81,-27,99,-21,99,-12, 99,0,true,true,0.5);
//        //intake
//        sleep(2000);
        }
        else{
            robot.goToPosition(0, -52, -5.5,90, 0.5);
//                robot.TurretSlidesToPosition(-9, 10, 0, 0.5);
//                sleep(1300);
            robot.FlipBasketArmToPosition(.45);
            sleep(700);
            robot.FlipBasketToPosition(0.0);
            //23, -17
            sleep(1000);
            robot.FlipBasketToPosition(0.4);
            robot.TurretSlidesToPosition(0, 0, 0, 0.5);
            robot.FlipBasketArmToPosition(0.00);
            robot.goToPosition(1, -45, 0,90, 0.5);
            robot.goToPosition(1, -8, 33-moved, 40, 0.4);
            robot.setMotorPowers(0.2);
            robot.spinCarouselAutonomousRed();
            robot.setMotorPowers(-0.3);
            sleep(500);
            robot.goToPosition(0, -6, 0-moved, 80, 0.4);
            robot.partOfPolySplineToPositionHead(0,33-moved,-8,0-moved,-6,-40-moved,-3.3,-69-moved,-3.3,true,true,0.5);
            robot.partOfPolySplineToPositionHead(0,0-moved,-6,-40-moved,-3.3,-69-moved,-3.3, -72-moved,-8,true,true,0.5);
        }
        //16,-60,-79   48,-24,-176    90,     81,-24,90    92,-24   92,-12
        stop();
    }
}