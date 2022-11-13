package org.firstinspires.ftc.teamcode.Tests;

import static org.firstinspires.ftc.teamcode.Old.Components.Chassis.EncoderChassis.xpos;
import static org.firstinspires.ftc.teamcode.Old.Components.Chassis.EncoderChassis.ypos;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.logger;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Robots.PwPRobot;

import java.util.ArrayList;
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
