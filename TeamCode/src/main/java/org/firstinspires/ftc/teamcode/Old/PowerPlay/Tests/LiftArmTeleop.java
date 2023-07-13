package org.firstinspires.ftc.teamcode.Old.PowerPlay.Tests;

import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.logger;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Old.PowerPlay.Components.LiftArm;
import org.firstinspires.ftc.teamcode.Robots.BasicRobot;
@Disabled
@TeleOp
public class LiftArmTeleop extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
       BasicRobot robot = new BasicRobot(this,true);
        LiftArm lfitArm = new LiftArm();
        waitForStart();
        while(opModeIsActive()){
            logger.loopcounter++;
            if (gamepad1.a) {
                lfitArm.raiseLiftArmToOuttake();
            }
            if (gamepad1.b) {
                lfitArm.lowerLiftArmToIntake();
            }
        }
    stop();
    }
}
