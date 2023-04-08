package org.firstinspires.ftc.teamcode.Tests;

import static org.firstinspires.ftc.teamcode.Old.Components.Chassis.EncoderChassis.xpos;
import static org.firstinspires.ftc.teamcode.Old.Components.Chassis.EncoderChassis.ypos;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.logger;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.op;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Components.Switch;
import org.firstinspires.ftc.teamcode.Robots.BasicRobot;
import org.firstinspires.ftc.teamcode.Robots.PwPRobot;

import java.util.ArrayList;
@Config
@Autonomous(name = "switchTest")
//@Disabled
public class switchTest extends LinearOpMode{

    public void runOpMode() {
        BasicRobot robot = new BasicRobot(this,false);
        Switch switcho = new Switch();
        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            logger.loopcounter++;
            telemetry.addData("switched?", switcho.isSwitched());
            telemetry.update();
        }

        logger.log("/RobotLogs/GeneralRobot", "Program stopped normally. ");

        stop();

    }
}
