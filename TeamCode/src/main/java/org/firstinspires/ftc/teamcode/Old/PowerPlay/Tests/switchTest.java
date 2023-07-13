package org.firstinspires.ftc.teamcode.Old.PowerPlay.Tests;

import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.logger;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Old.PowerPlay.Components.Switch;
import org.firstinspires.ftc.teamcode.Robots.BasicRobot;

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
