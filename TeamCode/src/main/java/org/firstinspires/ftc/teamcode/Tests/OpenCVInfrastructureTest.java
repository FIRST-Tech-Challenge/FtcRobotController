package org.firstinspires.ftc.teamcode.Tests;


import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Components.BasicChassis;
import org.firstinspires.ftc.teamcode.Robot;

import java.util.Arrays;
@Disabled

@Autonomous(name= "OpenCVInfrastructureTest", preselectTeleOp = "OneGPTeleop")
public class OpenCVInfrastructureTest extends LinearOpMode {
    @Override
    public void runOpMode(){
        Robot robot = new Robot(this, BasicChassis.ChassisType.ENCODER, true, false, 0);
        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        robot.setPosition(0,0,0);
        double barcode = robot.BlueElemTest(this,0,0);
        waitForStart();
        sleep(3000);
        double[] position = robot.BlueWarehouseScam();
        telemetry.addData("position",Arrays.toString(position));
        telemetry.update();
        sleep(100000);
        stop();
    }
}

