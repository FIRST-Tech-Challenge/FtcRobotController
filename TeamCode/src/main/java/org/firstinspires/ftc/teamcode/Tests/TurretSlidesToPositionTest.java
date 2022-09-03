package org.firstinspires.ftc.teamcode.Tests;


import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Components.BasicChassis;
import org.firstinspires.ftc.teamcode.BlackoutRobot;
@Disabled

@Autonomous(name= "TurretSlidesToPositionTest", preselectTeleOp = "TwoGPTeleop")
public class TurretSlidesToPositionTest extends LinearOpMode {
    @Override
    public void runOpMode(){
        BlackoutRobot robot = new BlackoutRobot(BasicChassis.ChassisType.ODOMETRY, true, false,90);
        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        robot.setPosition(0,2,0);
        waitForStart();

        robot.TurretSlidesToPosition(100.0, 100.0, 50.0, 0.5,false);

        stop();
    }
}