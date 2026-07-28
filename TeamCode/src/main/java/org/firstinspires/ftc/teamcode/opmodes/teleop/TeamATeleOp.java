package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.core.input.InputManager;
import org.firstinspires.ftc.teamcode.robots.teamA.TeamARobot;

/**
 * Team A's iterative TeleOp for four-wheel mecanum drive.
 *
 * <p>This OpMode maps gamepad input to Team A's public robot API. The robot and drivetrain
 * subsystem own the drive behavior and hardware access.</p>
 */
@TeleOp(name = "Team A Mecanum Drive", group = "Team A")
public class TeamATeleOp extends OpMode {
    private TeamARobot robot;
    private InputManager inputManager;

    @Override
    public void init() {
        robot = new TeamARobot();
        robot.initialize(hardwareMap);
        inputManager = new InputManager(gamepad1);

        telemetry.addData("Status", "Team A robot initialized");
        telemetry.update();
    }

    @Override
    public void start() {
        inputManager.update();
        robot.enableManualDrive();
    }

    @Override
    public void loop() {
        inputManager.update();

        if (inputManager.wasYJustPressed()) {
            robot.enableHeadingHold();
        } else if (inputManager.wasXJustPressed()) {
            robot.enableManualDrive();
        }

        robot.drive(-inputManager.getLeftStickY(), inputManager.getLeftStickX(),
                inputManager.getRightStickX());
        robot.update();
        publishDriveTelemetry();
    }

    @Override
    public void stop() {
        if (robot != null) {
            robot.stop();
        }
    }

    private void publishDriveTelemetry() {
        telemetry.addData("Drive State", robot.getDriveStateName());
        telemetry.addData("Forward", robot.getRequestedForward());
        telemetry.addData("Strafe", robot.getRequestedStrafe());
        telemetry.addData("Rotate", robot.getRequestedRotate());
        telemetry.addData("Front Left Power", robot.getFrontLeftMotorPower());
        telemetry.addData("Front Right Power", robot.getFrontRightMotorPower());
        telemetry.addData("Rear Left Power", robot.getRearLeftMotorPower());
        telemetry.addData("Rear Right Power", robot.getRearRightMotorPower());
        telemetry.update();
    }
}
