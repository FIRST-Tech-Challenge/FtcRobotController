package org.firstinspires.ftc.teamcode.Autonomous;


import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Components.BasicChassis;
import org.firstinspires.ftc.teamcode.Robot;
//cock warren like
@Autonomous(name= "BlueLeftP", preselectTeleOp = "OneGPTeleop")
public class BlueLeftP extends LinearOpMode {
    @Override
    public void runOpMode(){
        Robot robot = new Robot(this, BasicChassis.ChassisType.VSLAM, false, false);

        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        int position = robot.BlueElemTest(this,0,0);
        robot.setPosition(0,0,0);
        waitForStart();
        robot.goToPosition(0,-13,0,0,0.5);
        //Turret extension combined with rotation in such a way to achieve the current location to drop the loaded freight into the correct position by barcode
        if(position==1) {
            robot.TurretSlidesToPosition(1, 2.5, 0, 0.5);
            sleep(1300);
            robot.FlipBasketArmToPosition(.75);
            sleep(500);
            robot.FlipBasketToPosition(0.0);
            sleep(1000);
        }
        if(position==0){
            robot.TurretSlidesToPosition(3, 4.5, 0, 0.5);
            sleep(1300);
            robot.FlipBasketArmToPosition(.6);
            sleep(500);
            robot.FlipBasketToPosition(0.0);
            sleep(1000);
        }
        if(position==2){
            robot.TurretSlidesToPosition(7, 9.5, 0, 0.5);
            sleep(1300);
            robot.FlipBasketArmToPosition(.45);
            sleep(500);
            robot.FlipBasketToPosition(0.0);
            sleep(1000);
        }
        robot.FlipBasketToPosition(0.4);
        robot.TurretSlidesToPosition(0,0,0,0.5);
        sleep(1500);
        robot.FlipBasketArmToPosition(0.00);
        robot.turnInPlace(180,0.6);
        robot.goToPosition(0,-13.5,3,-100,0.5);
        robot.partOfPolySplineToPositionHead(0,3,-13.5,3,-13.5,5,-10, 7,-6,true,true,0.5);
        robot.partOfPolySplineToPositionHead(0,3,-13.5,5,-10, 7,-6,25,-6,true,true,0.5);
        robot.partOfPolySplineToPositionHead(0,5,-10, 7,-6,25,-6, 50,-6,true,true,0.5);
        robot.turnInPlace(90,0.5);
        robot.goToPosition(0,-2.5,35,90,0.5);

//        robot.partOfPolySplineToPositionHead(0,30,0,30,0,1,-2,0,-15,true,true,0.5);
//        robot.partOfPolySplineToPositionHead(0,30,0,1,-2,0,-15,0,-15,true,true,0.5);
        sleep(10000);
        stop();
    }
}