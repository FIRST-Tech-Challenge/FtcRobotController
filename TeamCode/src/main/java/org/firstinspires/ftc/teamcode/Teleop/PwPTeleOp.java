package org.firstinspires.ftc.teamcode.Teleop;

import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.logger;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robots.PwPRobot;

@TeleOp
public class PwPTeleOp extends LinearOpMode {
    PwPRobot robot;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new PwPRobot(this,true);
        robot.roadrun.setPoseEstimate(new Pose2d(41, 61, Math.toRadians(270)));
        robot.cv.observeStick();
        waitForStart();
        while(!isStopRequested()){
            logger.loopcounter++;
            robot.teleOp();
        }
        robot.stop();
    }
}
