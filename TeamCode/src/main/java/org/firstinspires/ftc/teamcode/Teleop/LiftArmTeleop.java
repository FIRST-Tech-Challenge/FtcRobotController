package org.firstinspires.ftc.teamcode.Teleop;

import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.logger;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robots.PwPRobot;

@TeleOp
public class LiftArmTeleop extends LinearOpMode {
    PwPRobot robot;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new PwPRobot(this,true);
        waitForStart();
        while(opModeIsActive()){
            logger.loopcounter++;
            if (gamepad1.a) {
                robot.raiseLiftArmToOuttake();
            }
            if (gamepad1.b) {
                robot.lowerLiftArmToIntake();
            }
        }
        robot.stop();
    }
}
