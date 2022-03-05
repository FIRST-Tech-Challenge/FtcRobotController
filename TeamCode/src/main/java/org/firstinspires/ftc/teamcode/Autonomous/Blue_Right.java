package org.firstinspires.ftc.teamcode.Autonomous;


import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Components.BasicChassis;
import org.firstinspires.ftc.teamcode.Robot;

@Autonomous(name= "Blue_Right_Regional", preselectTeleOp = "OneGPTeleop")
public class Blue_Right extends LinearOpMode {
    @Override
    public void runOpMode(){
        Robot robot = new Robot(this, BasicChassis.ChassisType.ODOMETRY, true, false);
        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        double cycletime = 15;

        robot.setPosition(0,2,0);
        waitForStart();

        robot.goToPosition(0,-5,0,85,0.6);
//      poop freight out of turret
        robot.goToPosition(0, -5,-26,85, 0.4);
//        robot.spinCarouselAutonomousBlue();
        robot.turnInPlace(180, 0.7);
        robot.partOfPolySplineToPositionHead(0, -26, -5, -26, -5, 0, -53, 26, -53, true, true, 0.5);
        robot.partOfPolySplineToPositionHead(0, -26, -5, 0, -53, 26, -53, -50, -53, true, true, 0.5);
        while(30-time>cycletime) {
            robot.goToPosition(0,-53,74, -90, 0.5);
            robot.goToPosition(0,-20,74, 0,0.5);
            double[] target = {0};//BLUEWAREHOUSEASCAM
            robot.goToPosition(0, target[1], target[0], 0, 0.5);
            robot.goToPosition(0,-53,74, -90, 0.5);
            robot.goToPosition(0,-53,50, -90, 0.5);
            //poop freight out of turret
        }
        robot.goToPosition(0,-53,74, 0,0.5);
        robot.goToPosition(0, -20, 74, 0, 0.6);
        stop();
    }
}