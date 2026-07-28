package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.core.input.InputManager;
import org.firstinspires.ftc.teamcode.robots.teamB.TeamBRobot;

/** Initial Team B TeleOp with cautious shared mecanum drive controls. */
@TeleOp(name = "Team B TeleOp", group = "Team B")
public class TeamBTeleOp extends OpMode {
    private TeamBRobot robot;
    private InputManager driverInput;

    @Override
    public void init() {
        robot = new TeamBRobot();
        robot.initialize(hardwareMap);
        driverInput = new InputManager(gamepad1);

        telemetry.addData("Status", "Team B robot initialized");
        telemetry.update();
    }

    @Override
    public void start() {
        driverInput.update();
        robot.enableManualDrive();
    }

    @Override
    public void loop() {
        driverInput.update();

        // Common drive mapping: left stick drives and strafes; right stick rotates.
        robot.drive(-driverInput.getLeftStickY(), driverInput.getLeftStickX(),
                driverInput.getRightStickX());

        // TODO: Add Team B mechanism mappings after its controls and hardware are defined.
        // TODO: Add Team B mode-selection mappings only when the team approves them.

        robot.update();
        publishTelemetry();
    }

    @Override
    public void stop() {
        if (robot != null) {
            robot.stop();
        }
    }

    private void publishTelemetry() {
        telemetry.addData("Drive State", robot.getDriveStateName());
        telemetry.addData("Intake State", robot.getIntakeStateName());
        telemetry.addData("Intake Available", robot.isIntakeAvailable());
        telemetry.addData("Vision State", robot.getVisionStateName());
        telemetry.addData("Vision Available", robot.isVisionAvailable());
        telemetry.update();
    }
}
