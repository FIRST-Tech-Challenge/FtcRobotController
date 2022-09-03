package org.firstinspires.ftc.teamcode.Autonomous;


import static org.firstinspires.ftc.teamcode.BasicRobot.op;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Components.BasicChassis;
import org.firstinspires.ftc.teamcode.BlackoutRobot;
//@Disabled

@Autonomous(name= "RedRightP", preselectTeleOp = "OneGPTeleop")
public class RedRightP extends LinearOpMode {
    @Override
    public void runOpMode(){
        BlackoutRobot robot = new BlackoutRobot(this, BasicChassis.ChassisType.ENCODER, false, false,90);
//        double[] turretTarget = {12+10.6,-24+16.2,0}; //{hubx-position*3/2,huby-position*3/2,1+7*position}
        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        robot.setPosition(0,0,0);
        robot.rotateToPosition(-80);
        waitForStart();
        robot.tseToPosition(0.8);
        resetStartTime();
        robot.rotateToPosition(5);
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
            robot.TurretSlidesToPosition(15.5, 10.0, 0, 0.5,false);
            robot.FlipBasketArmToPosition(0.6);
            robot.goToPosition(0, -14, 0, 0, 0.5);
            sleep(500);
            robot.FlipBasketToPosition(0.2);
            sleep(600);
        }
        else if (position==0) {
            robot.TurretSlidesToPosition(15, 11.5, 0, 0.5,false);
            robot.FlipBasketArmToPosition(0.9);
            robot.goToPosition(0, -14, 0, 0, 0.5);
            sleep(500);
            robot.FlipBasketToPosition(0.2);
            sleep(600);
        }
        else {
            robot.TurretSlidesToPosition(25, 18, 4, 0.5,false);
            robot.FlipBasketArmToPosition(0.4);
            robot.goToPosition(0, -14, 0, 0, 0.5);
            sleep(500);
            robot.FlipBasketToPosition(0.3);
            sleep(600);
            robot.FlipBasketToPosition(0.7);
        }
        robot.TurretSlidesToPosition(0 ,0,0,1.0,true);
        robot.FlipBasketArmToPosition(0.0);
        sleep(1000);
        robot.turnInPlace(60,1.0);
        robot.FlipBasketToPosition(0.8);
        robot.goToPosition(1,-6.75,13.5,45,0.5);
        robot.setMotorPowers(0.15);
        robot.spinCarouselAutonomousRed();
        robot.goToPosition(0,-28,18,0,0.5);
        robot.rotateToPosition(90);
        sleep(3000);

        stop();
    }
}