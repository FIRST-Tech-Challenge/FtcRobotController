package org.firstinspires.ftc.teamcode.Autonomous;


import static org.firstinspires.ftc.teamcode.BasicRobot.op;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Components.BasicChassis;
import org.firstinspires.ftc.teamcode.BlackoutRobot;
@Disabled
@Autonomous(name= "BlueRightPWarehouse", preselectTeleOp = "OneGPTeleop")
public class BlueRightPWarehouse extends LinearOpMode {
    @Override
    public void runOpMode(){
        BlackoutRobot robot = new BlackoutRobot(this, BasicChassis.ChassisType.VSLAM, false, false,90);
        sleep(1000);
        int position = robot.BlueElemTest(this,0,0);
        double[] turretTarget = {12+10.6,-24+16.2,0};//{hubx-position*3/2,huby-position*3/2,1+7*position}
        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        robot.setPosition(0,0,0);
        waitForStart();
        robot.goToPosition(0,-13.75,0,0,0.5);
        //Turret extension combined with rotation in such a way to achieve the current location to drop the loaded freight into the correct position by barcode
        if(position==0) {
            robot.TurretSlidesToPosition(-.2, .3, 0, 0.5,false);
            sleep(1300);
            robot.FlipBasketArmToPosition(.75);
            sleep(500);
            robot.FlipBasketToPosition(0.0);
            sleep(1000);
        }
        if(position==1){
            robot.TurretSlidesToPosition(-.75, 1.2, 0, 0.5,false);
            sleep(1300);
            robot.FlipBasketArmToPosition(.6);
            sleep(400);
            robot.FlipBasketToPosition(0.0);
            sleep(1000);
        }
        if(position==2){
            robot.TurretSlidesToPosition(-7.5, 9.0, 0, 0.5,false);
            sleep(1300);
            robot.FlipBasketArmToPosition(.45);
            sleep(300);
            robot.FlipBasketToPosition(0.0);
            sleep(1000);
        }
        robot.FlipBasketToPosition(0.4);
        sleep(1500);
        robot.TurretSlidesToPosition(0,0,0,0.5,false);
        sleep(1500);
        robot.FlipBasketArmToPosition(0.00);
        sleep(1500);
            robot.goToPosition(1, -10, -32, -50, 0.4);
            robot.setMotorPowers(0.25);
            robot.spinCarouselAutonomousBlue();
            robot.setMotorPowers(-0.3);
            sleep(1000);
            robot.goToPosition(1,-39,-32,-180,0.5);
//        robot.partOfPolySplineToPositionHead(0,-25.5,-5.6,-25.5,-5.6,12,-50,16,-38,true,true,0.5);
//        robot.partOfPolySplineToPositionHead(0,-25.5,-5.6,12,-50,35,-27,16,-38,true,true,0.5);
//        robot.turnInPlace(90,0.5);
//        robot.goToPosition(1,-25,81,90,1.0);
//        robot.partOfPolySplineToPositionHead(1,40,-27, 81,-27,99,-21,99,-12,true,true,0.5);
//        robot.partOfPolySplineToPositionHead(1, 81,-27,99,-21,99,-12, 99,0,true,true,0.5);
//        //intake
//        sleep(2000)
        //16,-60,-79   48,-24,-176    90,     81,-24,90    92,-24   92,-12
        stop();
    }
}