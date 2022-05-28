package org.firstinspires.ftc.teamcode.Autonomous;


import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Components.BasicChassis;
import org.firstinspires.ftc.teamcode.Robot;

@Autonomous(name= "BlueLeftP", preselectTeleOp = "OneGPTeleop")
public class BlueLeftP extends LinearOpMode {
    @Override
    public void runOpMode(){
        Robot robot = new Robot(this, BasicChassis.ChassisType.ENCODER, false, false, 0);

        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        int position = robot.RedElemTest(this,0,0);
        robot.setPosition(0,0,0);
        waitForStart();
        robot.goToPosition(0,-13,0,0,0.5);
        //Turret extension combined with rotation in such a way to achieve the current location to drop the loaded freight into the correct position by barcode
        if(position==2) {
            robot.TurretSlidesToPosition(.6, .9, 0, 0.5);
            sleep(1300);
            robot.FlipBasketArmToPosition(.75);
            sleep(500);
            robot.FlipBasketToPosition(0.0);
            sleep(1000);
        }
        if(position==0){
            robot.TurretSlidesToPosition(.75, 1.2, 0, 0.5);
            sleep(1300);
            robot.FlipBasketArmToPosition(.6);
            sleep(400);
            robot.FlipBasketToPosition(0.0);
            sleep(1000);
        }
        if(position==1){
            robot.TurretSlidesToPosition(7.5, 9.0, 0, 0.5);
            sleep(1300);
            robot.FlipBasketArmToPosition(.45);
            sleep(300);
            robot.FlipBasketToPosition(0.0);
            sleep(1000);
        }
        robot.FlipBasketToPosition(0.4);
        sleep(1500);
         robot.TurretSlidesToPosition(0,0,0,0.5);
        sleep(1500);
        robot.FlipBasketArmToPosition(0.00);
        sleep(1500);

        stop();
    }
}