package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.powerplay.PowerPlayRobot;
import org.firstinspires.ftc.teamcode.robotbase.GamepadExEx;


@TeleOp(name = "PowerPlay", group = "TeleOP")
public class PowerPlayTeleOp extends CommandOpMode {
    private PowerPlayRobot robot;

    @Override
    public void initialize() {
        GamepadExEx driverOp = new GamepadExEx(gamepad1);
        GamepadExEx toolOp = new GamepadExEx(gamepad2);

        robot = new PowerPlayRobot(hardwareMap, telemetry, driverOp, toolOp);
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
