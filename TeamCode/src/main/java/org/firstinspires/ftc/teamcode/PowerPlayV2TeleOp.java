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
        GamepadEx driverOp = new GamepadEx(gamepad1);
        GamepadEx toolOp = new GamepadEx(gamepad2);

        robot = new PowerPlayRobotV2(hardwareMap, telemetry, driverOp, toolOp);
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