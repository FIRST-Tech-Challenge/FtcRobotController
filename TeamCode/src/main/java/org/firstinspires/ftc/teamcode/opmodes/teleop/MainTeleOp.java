package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.util.RobotLog;

// Main TeleOp mode for driving the robot using gamepad controls.
// Class MainTeleOp represents the main teleoperated mode for the robot.
@TeleOp(name = "Main TeleOp", group = "Main")
public class MainTeleOp extends OpMode {

    // The robot instance that this teleop mode will control.
    private Robot robot;

    // Initializes the robot and its subsystems when the teleop mode starts.
    // hardwareMap and telemetry are passed to the Robot constructor from the OpMode.
    @Override
    public void init() {
        RobotLog.i("MainTeleOp: Initializing robot");
        robot = new Robot(hardwareMap, telemetry);
        robot.init();
        RobotLog.i("MainTeleOp: Robot initialized");
    }

    // Reads the gamepad inputs and updates the robot's subsystems accordingly.
    @Override
    public void loop() {
        // Left stick drives and strafes, right stick turns
        RobotLog.i("MainTeleOp: Driving with forward=" + -gamepad1.left_stick_y + " strafe=" + gamepad1.left_stick_x + " turn=" + gamepad1.right_stick_x);
        robot.drive.drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        robot.launcher.spinUp(1150);
        robot.vision.TipDetect(42,45);
        // TODO: launcher, intake, and aiming controls
        robot.update();
    }

    // Stops the robot and its subsystems when the teleop mode ends.
    @Override
    public void stop() {
        RobotLog.i("MainTeleOp: Stopping robot");
        robot.stop();
    }
}
