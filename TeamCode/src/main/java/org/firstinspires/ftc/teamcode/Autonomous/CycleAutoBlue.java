package org.firstinspires.ftc.teamcode.Autonomous;


import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Components.BasicChassis;
import org.firstinspires.ftc.teamcode.Robot;

@Autonomous(name= "CycleAuto", preselectTeleOp = "OneGPTeleop")
public class CycleAutoBlue extends LinearOpMode {
    @Override
    public void runOpMode() {
        Robot robot = new Robot(this, BasicChassis.ChassisType.ENCODER, false, true,0);
        robot.rotateToPosition(-88);
        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        int position = robot.RedElemTest(this, 0, 0);
        double[] turretTarget = {12 + 10.6, -24 + 16.2, 0};//{hubx-position*3/2,huby-position*3/2,1+7*position}
        robot.setPosition(0, 0, 0);
        waitForStart();
        robot.tseToPosition(0.6);
        resetStartTime();
        //Turret extension combined with rotation in such a way to achieve the current location to drop the loaded freight into the correct position by barcode
        if (position == 0) {
            robot.TurretSlidesToPosition(-19, 9, 0, 1.0,false);
            sleep(700);
            robot.FlipBasketArmToPosition(0.8);
            sleep(700);
            robot.FlipBasketToPosition(0.18);
            sleep(300);
        }
        if (position == 1) {
            robot.TurretSlidesToPosition(-20, 8, 2, 1.0,false);
            sleep(1000);
            robot.FlipBasketArmToPosition(0.6);
            sleep(250);
            robot.FlipBasketToPosition(0.18);
            sleep(400);
        }
        if (position == 2) {
            robot.TurretSlidesToPosition(-30, 15, 6, 1.0,false);
            sleep(1000);
            robot.FlipBasketArmToPosition(.47);
            sleep(250);
            robot.FlipBasketToPosition(0.18);
            sleep(400);
        }
//        sleep(5000);
//        robot.FlipBasketToPosition(0.58);
        double times=0;
        boolean sheesher = true;
        while (getRuntime()<28){
            sheesher = robot.autoIntake(0.8, 10,times);
            if(sheesher) {
                robot.FlipBasketArmToPosition(.55);
                robot.FlipBasketToPosition(0.18);
                robot.stopIntake();
                sleep(400);
                times++;
            }
        }

//        robot.goToPosition(1, -11.5, -28, -30, 0.8);
//        robot.FlipBasketArmToPosition(0.00);
//        robot.setMotorPowers(0.2);
//        robot.spinCarouselAutonomousBlue();
//        robot.goToPosition(1, -3.2, 0, 80, 0.8);
//        robot.partOfPolySplineToPositionHead(1, -28, -15, 0, -4.0, 40, -2.0, 69, -2.0, true, true, 0.8);
//        robot.flipIntakeToPosition(0.0);
//        robot.partOfPolySplineToPositionHead(1, 0, -4.0, 40, -2.0, 70, -2.0, 78, -2.0, true, true, 0.8);
//        while (getRuntime()<26){
//            robot.autoIntake(0.8, 5);
//            robot.FlipBasketArmToPosition(0.45);
//            sleep(300);
//            robot.FlipBasketToPosition(0.18);
//        }
//        robot.autoPark(74,-2.4,0.8);
//
////        while(getRuntime()<24) {
////            robot.autoIntake(0.4,5);
//////            robot.goToPosition(0, -1, 5, 90, 0.5);
//////            robot.TurretSlidesToPosition(9.0, 7.5, 0, 0.5);
//////            sleep(1300);
//////            robot.FlipBasketArmToPosition(.45);
//////            sleep(300);
//////            robot.FlipBasketToPosition(0.0);
//////            sleep(1000);
//////            robot.goToPosition(1, -1, 30, 90, 0.5);
////        }
////        robot.partOfPolySplineToPositionHead(0,30,0,30,0,1,-2,0,-15,true,true,0.5);
////        robot.partOfPolySplineToPositionHead(0,30,0,1,-2,0,-15,0,-15,true,true,0.5);
        stop();
    }
}