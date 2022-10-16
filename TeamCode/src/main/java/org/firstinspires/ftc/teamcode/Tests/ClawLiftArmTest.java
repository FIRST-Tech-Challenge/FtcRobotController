package org.firstinspires.ftc.teamcode.Tests;

import static org.firstinspires.ftc.teamcode.Old.Components.Chassis.EncoderChassis.xpos;
import static org.firstinspires.ftc.teamcode.Old.Components.Chassis.EncoderChassis.ypos;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.logger;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Robots.PwPRobot;

import java.util.ArrayList;

@Config
@Autonomous(name = "ClawLiftArmTest")
//@Disabled
public class ClawLiftArmTest extends LinearOpMode{

    public void runOpMode() {

        telemetry.addData("Status", "Before new Robot");
        telemetry.update();

        PwPRobot robot = new PwPRobot(this, true);

        telemetry.addData("Status", "Ready to go");
        telemetry.update();

        FtcDashboard dashboard = FtcDashboard.getInstance();
        waitForStart();

        logger.log("/RobotLogs/GeneralRobot", "Running: ClawTest\n");



        TelemetryPacket packet = new TelemetryPacket();
        double x = xpos - 8.75;
        double y = ypos - 6.25;



        packet.fieldOverlay().setFill("blue").fillRect(x, y, 15, 15);
        telemetry.addData("status", "waiting for start command...");
        telemetry.addData("xpos",x);
        telemetry.addData("ypos",y);
        dashboard.sendTelemetryPacket(packet);
        telemetry.update();

        while (opModeIsActive() && !isStopRequested()) {
            logger.loopcounter++;
//            robot.closeClaw();
//            sleep(2000);
            robot.raiseLiftArmToOuttake();
            sleep(3000);
//            robot.openClaw();
//            sleep(2000);
            robot.lowerLiftArmToIntake();
            sleep(3000);

            robot.setFirstLoop(false);
        }

        logger.log("/RobotLogs/GeneralRobot", "Program stopped normally. ");

        idle();

    }
}
