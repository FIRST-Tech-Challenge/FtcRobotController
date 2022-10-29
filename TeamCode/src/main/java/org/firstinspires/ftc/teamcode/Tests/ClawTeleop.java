package org.firstinspires.ftc.teamcode.Tests;

import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.logger;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robots.PwPRobot;

@TeleOp
public class ClawTeleop extends LinearOpMode {
    PwPRobot robot;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new PwPRobot(this,true);
        waitForStart();
        while(opModeIsActive()){
            logger.loopcounter++;
            if (gamepad1.a) {
                robot.openClaw();
            }
            if (gamepad1.b) {
                robot.closeClaw();
            }
        }
        robot.stop();
    }
}
