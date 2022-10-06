package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robots.PwPRobot;

public class PwPTeleOp extends LinearOpMode {
    PwPRobot robot;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new PwPRobot(this,true);
        waitForStart();
        while(opModeIsActive()){
            robot.teleOp();
        }
        robot.stop();
    }
}
