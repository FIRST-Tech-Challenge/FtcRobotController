package org.firstinspires.ftc.teamcode.Tests;


import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Components.BasicChassis;
import org.firstinspires.ftc.teamcode.Robot;

@Autonomous(name= "OpenCVInfrastructureTest", preselectTeleOp = "OneGPTeleop")
public class OpenCVInfrastructureTest extends LinearOpMode {
    @Override
    public void runOpMode(){
        Robot robot = new Robot(this, BasicChassis.ChassisType.ODOMETRY, true, false);
        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        robot.rotateToPosition(-10);
        robot.setPosition(0,0,0);
        int position = robot.BlueElemTest(this,0,0);

        waitForStart();
        sleep(3000);
        telemetry.update();
        sleep(100000);
        stop();
    }
}

