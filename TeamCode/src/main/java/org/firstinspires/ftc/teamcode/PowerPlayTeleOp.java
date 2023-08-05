package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Slidy_PPV2.SlidyRobot;
import org.inventors.ftc.robotbase.GamepadExEx;
import org.inventors.ftc.robotbase.MotorExEx;

@TeleOp(name = "PowerPlayBucharest2023", group = "Final TeleOPs")
public class PowerPlayTeleOp extends CommandOpMode {
    private SlidyRobot Slidy;

    @Override
    public void initialize() {
        GamepadExEx driverOp = new GamepadExEx(gamepad1);
        GamepadExEx toolOp = new GamepadExEx(gamepad2);

        Slidy = new SlidyRobot(hardwareMap, telemetry, driverOp, toolOp);
    }

    @Override
    public void run() {
        super.run();
        // TODO: Make telemetry subsystem/command and remove this function
        Slidy.telemetryUpdate();
        Slidy.dashboardTelemetryUpdate();
    }
}