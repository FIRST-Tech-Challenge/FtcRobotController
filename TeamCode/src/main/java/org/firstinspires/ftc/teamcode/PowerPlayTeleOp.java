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
        robot = new PowerPlayRobot(hardwareMap, telemetry, gamepad1, gamepad2);
    }

    @Override
    public void run() {
        super.run();
        telemetry.addData("Raw Stick Value: ", gamepad1.left_stick_y);
        telemetry.addData("Raw Stick Value: ", gamepad1.right_stick_y);
        // TODO: Make telemetry subsystem/command and remove this function
        robot.telemetryUpdate();
        robot.dashboardTelemetryUpdate();
    }
}