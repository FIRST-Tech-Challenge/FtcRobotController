package org.firstinspires.ftc.teamcode.Autonomous;


import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Components.BasicChassis;
import org.firstinspires.ftc.teamcode.Robot;

@Autonomous(name= "BlueRightPWarehouse", preselectTeleOp = "OneGPTeleop")
public class BlueRightPWarehouse extends LinearOpMode {
    @Override
    public void runOpMode(){
        Robot robot = new Robot(this, BasicChassis.ChassisType.VSLAM, false, false);
        sleep(1000);
        int position = robot.BlueElemTest(this,0,0);
        double[] turretTarget = {12+10.6,-24+16.2,0};//{hubx-position*3/2,huby-position*3/2,1+7*position}
        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        robot.setPosition(0,0,0);
        waitForStart();
        robot.TurretSlidesToPosition(0,0,0,0.4);
        if(position!=2) {
            robot.goToPosition(0, -15, -10, -49, 0.5);
        if(position==1) {
            robot.TurretSlidesToPosition(0, 20, 0, 0.5);
            sleep(1300);
            robot.FlipBasketArmToPosition(.6);
            sleep(500);
            robot.FlipBasketToPosition(0.0);
            sleep(1000);
        }
        if(position==0) {
            robot.TurretSlidesToPosition(0, 14.3, 0, 0.5);
            sleep(1300);
            robot.FlipBasketArmToPosition(.75);
            sleep(500);
            robot.FlipBasketToPosition(0.0);
            sleep(1000);
        }
            robot.FlipBasketToPosition(0.4);
            robot.TurretSlidesToPosition(0, 0, 0, 0.5);
            robot.FlipBasketArmToPosition(0.00);
            robot.goToPosition(1, -8, -33, -40, 0.4);
            robot.setMotorPowers(0.2);
            robot.spinCarouselAutonomousBlue();
            robot.setMotorPowers(-0.3);
            sleep(500);
            robot.goToPosition(0, -6, 0, -80, 0.4);
            robot.partOfPolySplineToPositionHead(0,-33,-8,0,-6,40,-3,69,-3,true,true,0.5);
            robot.partOfPolySplineToPositionHead(0,0,-6,40,-3,74,-3, 74,-8,true,true,0.5);
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
            robot.goToPosition(0, -24, 4.5,-33, 0.5);
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
            robot.goToPosition(1, -8, -33, -40, 0.4);
            robot.setMotorPowers(0.2);
            robot.spinCarouselAutonomousBlue();
            robot.setMotorPowers(-0.3);
            sleep(500);
            robot.goToPosition(0, -6, 0, -80, 0.4);
            robot.partOfPolySplineToPositionHead(0,-33,-8,0,-6,40,-3,69,-3,true,true,0.5);
            robot.partOfPolySplineToPositionHead(0,0,-6,40,-3,74,-3, 74,-8,true,true,0.5);
        }
        //16,-60,-79   48,-24,-176    90,     81,-24,90    92,-24   92,-12
        stop();
    }
}