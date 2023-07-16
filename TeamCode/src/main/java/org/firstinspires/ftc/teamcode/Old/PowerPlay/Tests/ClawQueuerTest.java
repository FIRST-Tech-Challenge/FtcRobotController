package org.firstinspires.ftc.teamcode.Old.PowerPlay.Tests;

import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.logger;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Old.PowerPlay.Robot.PwPRobot;

@Disabled
@Config
@Autonomous(name = "ClawQueuerTest")
//@Disabled
public class ClawQueuerTest extends LinearOpMode{

    public void runOpMode() {
        PwPRobot robot = new PwPRobot(this, true);
        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            logger.loopcounter++;
//            robot.toggleClawPosition(false);
            for (int i = 0; i < 5; i++) {
                robot.closeClaw(false);
                robot.raiseLiftArmToOuttake(false);
                robot.openClaw(false);
                robot.lowerLiftArmToIntake(false);
            }
            robot.setFirstLoop(false);
        }

        logger.log("/RobotLogs/GeneralRobot", "Program stopped normally. ");

        robot.stop();

    }
}
