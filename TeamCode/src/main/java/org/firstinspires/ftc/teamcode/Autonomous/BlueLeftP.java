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
        robot.setPosition(0,0,0);
        waitForStart();
//        robot.TurretSlidesToPosition(-11.5,10.0,0,0.5);
        robot.goToPosition(0,-15,0,15,0.5);
        //Turret extension combined with rotation in such a way to achieve the current location to drop the loaded freight into the correct position by barcode
        sleep(2000);
//        robot.FlipBasketArmToPosition(.5);
//        sleep(1000);
//        robot.FlipBasketToPosition(0.01);
//        sleep(1000);
//        robot.FlipBasketToPosition(0.5);
//        sleep(500);
//        robot.TurretSlidesToPosition(0,0,0,0.5);
        robot.goToPosition(1,-5,0,80,0.5);
        robot.partOfPolySplineToPositionHead(1,0,-15,0,-15,1,-2, 25,0,true,true,0.5);
        robot.partOfPolySplineToPositionHead(1,0,-15,1,-2, 25,0,50,0,true,true,0.5);
        robot.turnInPlace(90,0.5);
        robot.TurretReset(0.5);
        robot.autoIntake(0.3,5);
        while(getRuntime()<24) {
            robot.goToPosition(0, -1, 5, 90, 0.5);
            //drop
            sleep(2000);
            robot.goToPosition(1, -1, 30, 90, 0.5);
            //robot.autoIntake(0.3,5);
        }
//        robot.partOfPolySplineToPositionHead(0,30,0,30,0,1,-2,0,-15,true,true,0.5);
//        robot.partOfPolySplineToPositionHead(0,30,0,1,-2,0,-15,0,-15,true,true,0.5);
        sleep(10000);
        stop();
    }
}