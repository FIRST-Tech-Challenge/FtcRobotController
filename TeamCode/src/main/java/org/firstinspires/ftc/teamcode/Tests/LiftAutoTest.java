package org.firstinspires.ftc.teamcode.Tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Components.Lift;
import org.firstinspires.ftc.teamcode.Robots.PwPRobot;

@Config
@Autonomous(name = "LiftAutoTest")

public class LiftAutoTest extends LinearOpMode{
    public void runOpMode() {
        PwPRobot robot = new PwPRobot(this, false);
        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {
            robot.liftToPosition(Lift.LiftConstants.LIFT_MED_JUNCTION);
            robot.liftToPosition(Lift.LiftConstants.LIFT_LOW_JUNCTION);
            robot.liftToPosition(Lift.LiftConstants.LIFT_MED_JUNCTION);
            robot.liftToPosition(Lift.LiftConstants.LIFT_HIGH_JUNCTION);
            robot.liftToPosition(1000);
            robot.liftToPosition(1500);
            robot.liftToPosition(500);
            robot.liftToPosition(0);
            robot.setFirstLoop(false);
        }
    }
}
