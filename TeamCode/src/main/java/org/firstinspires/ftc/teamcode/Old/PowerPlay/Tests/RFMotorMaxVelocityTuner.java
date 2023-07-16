package org.firstinspires.ftc.teamcode.Old.PowerPlay.Tests;

import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.logger;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Components.RFModules.Devices.RFMotor;
import org.firstinspires.ftc.teamcode.Robots.BasicRobot;

@Config
@Autonomous(name = "RFMotorMaxVelocityTuner")

public class RFMotorMaxVelocityTuner extends LinearOpMode {
    public void runOpMode(){
        BasicRobot robot = new BasicRobot(this,false);
        double maxTick = 1690, minTick=0,avg1 = 0, avg2 =0, loopNums=0;
        RFMotor rfMotor = new RFMotor("liftMotor", DcMotor.RunMode.RUN_WITHOUT_ENCODER,true, maxTick,minTick);
        rfMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        waitForStart();
        double maxVelocity=0;
        while(rfMotor.getCurrentPosition()<maxTick){
            rfMotor.setPower(1.0);
            double velocity = rfMotor.getVelocity();
            if(velocity>maxVelocity){
                maxVelocity=velocity;
            }
            logger.log("/RobotLogs/GeneralRobot", ""+velocity);
        }
        logger.log("/RobotLogs/GeneralRobot", "maxVelocityt: "+maxVelocity);
        while(rfMotor.getCurrentPosition()>minTick){
            rfMotor.setPower(-0.3);
        }

    }

}
