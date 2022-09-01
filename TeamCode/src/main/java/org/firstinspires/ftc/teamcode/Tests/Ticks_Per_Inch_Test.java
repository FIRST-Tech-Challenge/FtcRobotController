package org.firstinspires.ftc.teamcode.Tests;


import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Components.BasicChassis;
import org.firstinspires.ftc.teamcode.BlackoutRobot;
@Disabled

@Autonomous(name= "Ticks_Per_Inch_Testing", preselectTeleOp = "TwoGPTeleop")
public class Ticks_Per_Inch_Test extends LinearOpMode {
    @Override
    public void runOpMode(){
        BlackoutRobot robot = new BlackoutRobot(this, BasicChassis.ChassisType.ODOMETRY, true, false,90);
        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        robot.setPosition(0,0,0);
        waitForStart();



        stop();
    }
}