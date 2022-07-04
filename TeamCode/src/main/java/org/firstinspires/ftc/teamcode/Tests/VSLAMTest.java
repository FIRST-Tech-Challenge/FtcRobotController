package org.firstinspires.ftc.teamcode.Tests;


import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Components.BasicChassis;
import org.firstinspires.ftc.teamcode.Robot;
@Disabled
@Autonomous(name= "VSLAMTest", preselectTeleOp = "OneGPTeleop")
public class VSLAMTest extends LinearOpMode {
    @Override
    public void runOpMode(){
        Robot robot = new Robot(this, BasicChassis.ChassisType.VSLAM, true, false,90);
        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        waitForStart();
        robot.goToPosition(1, 48,0,90,0.5);
        robot.goToPosition(1, 48,24,200,0.5);
        robot.goToPosition(1, 0,0,90,0.5);
        robot.goToPosition(1, 0,24,-20,0.5);
        robot.goToPosition(1, 48,0,90,0.5);
        robot.goToPosition(1, 48,24,180,0.5);
        robot.goToPosition(1,0,24,-90,0.5);
        robot.goToPosition(1, 0,0,-90,0.5);
        sleep(5000);
        stop();
    }
}

