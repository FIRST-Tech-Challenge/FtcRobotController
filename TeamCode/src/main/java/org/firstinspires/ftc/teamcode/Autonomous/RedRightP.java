package org.firstinspires.ftc.teamcode.Autonomous;


import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Components.BasicChassis;
import org.firstinspires.ftc.teamcode.Robot;
//@Disabled

@Autonomous(name= "RedRightP", preselectTeleOp = "OneGPTeleop")
public class RedRightP extends LinearOpMode {
    @Override
    public void runOpMode(){
        Robot robot = new Robot(this, BasicChassis.ChassisType.ENCODER, false, false,90);
//        double[] turretTarget = {12+10.6,-24+16.2,0}; //{hubx-position*3/2,huby-position*3/2,1+7*position}
        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        robot.setPosition(0,0,0);
        robot.rotateToPosition(-80);
        waitForStart();
        robot.tseToPosition(0.6);
        resetStartTime();
        robot.rotateToPosition(3);
        sleep(2000);
        int position = robot.BlueElemTest(this,0,0);
        while(getRuntime()<8){
            sleep(100);
        }

        if (position == 0) {
            position = 1;
        }
        else if (position == 1) {
            position = 0;
        }
        telemetry.addData("position:", position);
        telemetry.update();
        if(position==1) {
            robot.TurretSlidesToPosition(12.5, 10.0, 0, 0.5);
            robot.goToPosition(0, -14, 0, 0, 0.5);
            robot.FlipBasketArmToPosition(0.6);
            sleep(900);
            robot.FlipBasketToPosition(0.2);
            sleep(400);
        }
        else if (position==0) {
            robot.TurretSlidesToPosition(10, 8.5, 0, 0.5);
            robot.goToPosition(0, -14, 0, 0, 0.5);
            robot.FlipBasketArmToPosition(0.9);
            sleep(900);
            robot.FlipBasketToPosition(0.2);
            sleep(500);
        }
        else {
            robot.TurretSlidesToPosition(21, 14, 0, 0.5);
            robot.goToPosition(0, -14, 0, 0, 0.5);
            robot.FlipBasketArmToPosition(0.4);
            sleep(800);
            robot.FlipBasketToPosition(0.2);
            sleep(300);
        }
        robot.TurretSlidesToPosition(0 ,0,0,1.0);
        robot.FlipBasketArmToPosition(0.0);
        sleep(1000);
        robot.turnInPlace(60,1.0);
        robot.FlipBasketToPosition(0.8);
        robot.goToPosition(1,-7.75,15.5,40,0.5);
        robot.setMotorPowers(0.18);
        robot.spinCarouselAutonomousRed();
        robot.goToPosition(0,-27,18,0,0.5);
        robot.rotateToPosition(90);
        sleep(3000);

        stop();
    }
}