// DiffySwerveTeleOp.java
package org.firstinspires.ftc.teamcode.TeleOps.MainCode.TeleOps.TeleOps.Drivebases.Swerve.DiffySwerve;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class BetterDiffyTeleOp extends CommandOpMode {

    private ModuleLeft leftModule;
    private ModuleRight rightModule;
    private GamepadEx driverGamepad;

    @Override
    public void initialize() {
        leftModule = new ModuleLeft(hardwareMap, telemetry);
        rightModule = new ModuleRight(hardwareMap, telemetry);

        driverGamepad = new GamepadEx(gamepad1);

        // Set PID controllers and other components if needed
    }

    @Override
    public void run() {
        // Schedule the swerve drive command
        schedule(new DiffySwerveDriveCommand(leftModule, rightModule, driverGamepad));

        // Keep updating the scheduler to execute commands
        CommandScheduler.getInstance().run();
    }
}
