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
        Robot robot = new Robot(this, BasicChassis.ChassisType.ENCODER, false, false);
//        double[] turretTarget = {12+10.6,-24+16.2,0}; //{hubx-position*3/2,huby-position*3/2,1+7*position}
        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        robot.setPosition(0,0,0);
        robot.toggleTSEPosition();
        robot.rotateToPosition(-80);
        waitForStart();
        resetStartTime();
        robot.rotateToPosition(5);
        sleep(2000);
        int position = robot.RedElemTest(this,0,0);
        while(getRuntime()<8){
            sleep(100);
        }
        telemetry.addData("position:", position);
        telemetry.update();
        if(position==1) {
            robot.TurretSlidesToPosition(13.0, 8.5, 0, 0.5);
            robot.goToPosition(0, -20, 0, 0, 0.5);
            robot.FlipBasketArmToPosition(0.6);
            sleep(500);
            robot.FlipBasketToPosition(0.2);
            sleep(300);
        }
        else if (position==0) {
            robot.TurretSlidesToPosition(8.0, 5.5, 0, 0.5);
            robot.goToPosition(0, -20, 0, 0, 0.5);
            robot.FlipBasketArmToPosition(0.9);
            sleep(500);
            robot.FlipBasketToPosition(0.2);
            sleep(300);
        }
        else {
            robot.TurretSlidesToPosition(17.5, 9.5, 0, 0.5);
            robot.goToPosition(0, -20, 0, 0, 0.5);
            robot.FlipBasketArmToPosition(0.4);
            sleep(300);
            robot.FlipBasketToPosition(0.2);
            sleep(300);
        }
        robot.TurretSlidesToPosition(0 ,0,0,1.0);
        robot.FlipBasketArmToPosition(0.0);
        robot.turnInPlace(60,1.0);
        robot.FlipBasketToPosition(0.8);
        robot.goToPosition(1,-7,18,40,0.5);
        robot.setMotorPowers(0.18);
        robot.spinCarouselAutonomousRed();
        robot.goToPosition(0,-29,18,0,0.5);
        robot.rotateToPosition(90);
        sleep(500);

        stop();
    }
}