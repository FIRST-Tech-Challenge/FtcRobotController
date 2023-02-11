package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.powerplayV2.PowerPlayRobotV2;


@TeleOp(name = "PowerPlayV2", group = "TeleOP")
public class PowerPlayV2TeleOp extends CommandOpMode {
    private PowerPlayRobotV2 robot;

    @Override
    public void initialize() {
        robot = new PowerPlayRobotV2(hardwareMap, telemetry, gamepad1, gamepad2);
    }

    @Override
    public void run() {
        super.run();
        // TODO: Make telemetry subsystem/command and remove this function
        robot.telemetryUpdate();
        robot.dashboardTelemetryUpdate();
    }
}