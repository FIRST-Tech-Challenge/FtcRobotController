package org.firstinspires.ftc.teamcode.Old.PowerPlay.Tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Old.PowerPlay.Components.Lift;
import org.firstinspires.ftc.teamcode.Old.PowerPlay.Robot.PwPRobot;

@Config
@Autonomous(name = "LiftAutoTest")
@Disabled
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
