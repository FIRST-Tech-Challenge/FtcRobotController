package org.firstinspires.ftc.teamcode.Tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Components.RFModules.Devices.RFMotor;
import org.firstinspires.ftc.teamcode.Robots.BasicRobot;

@Config
@Autonomous(name = "RFMotorkATuner")

public class RFMotorkATuner extends LinearOpMode {
    public void runOpMode(){
        BasicRobot robot = new BasicRobot(this,false);
        double maxTick = 1200, minTick=0,avg1 = 0, avg2 =0, loopNums=0;
        RFMotor rfMotor = new RFMotor("liftMotor", DcMotor.RunMode.RUN_WITHOUT_ENCODER,true, maxTick,minTick);
        rfMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        waitForStart();
        while(opModeIsActive()){
            rfMotor.setPosition(maxTick);
            while(!rfMotor.atTargetPosition()) {
                rfMotor.setPosition(maxTick);
            }
            rfMotor.setPosition(minTick);
            while(!rfMotor.atTargetPosition()) {
                rfMotor.setPosition(minTick);
            }
        }

    }

}
