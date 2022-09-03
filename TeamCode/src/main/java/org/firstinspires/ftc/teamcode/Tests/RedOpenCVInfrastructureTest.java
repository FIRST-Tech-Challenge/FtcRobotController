package org.firstinspires.ftc.teamcode.Tests;


import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Components.BasicChassis;
import org.firstinspires.ftc.teamcode.BlackoutRobot;

@Autonomous(name= "RedOpenCVInfrastructureTest", preselectTeleOp = "OneGPTeleop")
public class RedOpenCVInfrastructureTest extends LinearOpMode {
    @Override
    public void runOpMode(){
        BlackoutRobot robot = new BlackoutRobot(BasicChassis.ChassisType.ENCODER, true, false,90);
        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        robot.rotateToPosition(88);
        sleep(2000);
        robot.rotateToPosition(5);
        sleep(2000);
        robot.setPosition(0,0,0);
        int position = robot.BlueElemTest(this,0,0);

        waitForStart();
        sleep(3000);
        telemetry.update();
        sleep(100000);
        stop();
    }
}

