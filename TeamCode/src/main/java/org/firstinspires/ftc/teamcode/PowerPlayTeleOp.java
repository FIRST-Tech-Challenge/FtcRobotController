package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.powerplay.PowerPlayRobot;


@TeleOp(name = "PowerPlay", group = "TeleOP")
public class PowerPlayTeleOp extends CommandOpMode {
    private PowerPlayRobot robot;

    @Override
    public void initialize() {
        GamepadEx driverOp = new GamepadEx(gamepad1);
        GamepadEx toolOp = new GamepadEx(gamepad2);

        robot = new PowerPlayRobot(hardwareMap, telemetry, driverOp, toolOp);
    }

    @Override
    public void run() {
        super.run();
        // TODO: Make telemetry subsystem/command and remove this function
        robot.telemetryUpdate();
        robot.dashboardTelemetryUpdate();
    }
}