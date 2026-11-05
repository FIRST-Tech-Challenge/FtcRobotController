package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.LauncherConfig;
import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.util.RobotLog;

/*
 * Main driver-controlled mode.
 * Left stick: drive/strafe. Right stick: turn.
 * Right trigger: feed (hold). Right bumper: toggle intake.
 * Left bumper (hold): run the flywheel backward to clear a stuck ball.
 * Launcher spins continuously at LauncherConfig.DEFAULT_RPM so it's always ready to feed.
 * Controls ported from the tested SuperTestTeleop bench test.
 * TODO: aiming controls (button mapping not decided yet)
 */
@TeleOp(name = "Main TeleOp", group = "Main")
public class MainTeleOp extends OpMode {

    private Robot robot;

    @Override
    public void init() {
        RobotLog.i("MainTeleOp: Initializing robot");
        robot = new Robot(hardwareMap, telemetry);
        robot.init();
        RobotLog.i("MainTeleOp: Robot initialized");
    }

    @Override
    public void loop() {
        robot.drive.drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

        if (gamepad1.left_bumper) {
            // Held: run the flywheel backward to clear a stuck ball.
            robot.launcher.reverse(RobotConstants.LAUNCHER_REVERSE_POWER);
        } else {
            robot.launcher.spinUp(LauncherConfig.DEFAULT_RPM);
            if (gamepad1.right_trigger >= 0.3) {
                robot.launcher.feed();
            } else {
                robot.launcher.retractFeeder();
            }
        }

        if (gamepad1.rightBumperWasPressed()) {
            robot.intake.toggle();
        }

        telemetry.addData("Hive tipped",
                robot.vision.isHiveTipped(RobotConstants.TAG_BLUE_AUDIENCE_MIN, RobotConstants.TAG_BLUE_AUDIENCE_MAX));
        robot.update();
    }

    // Stops the robot and its subsystems when the teleop mode ends.
    @Override
    public void stop() {
        RobotLog.i("MainTeleOp: Stopping robot");
        robot.stop();
    }
}
