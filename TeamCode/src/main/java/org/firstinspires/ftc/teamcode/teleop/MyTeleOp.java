package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.command.ArmMovePosition;
import org.firstinspires.ftc.teamcode.command.TeleopDrive;
import org.firstinspires.ftc.teamcode.subsystem.Arm;
import org.firstinspires.ftc.teamcode.subsystem.Drive;

@TeleOp
public class MyTeleOp extends CommandOpMode {
    private final FtcDashboard dashboard = FtcDashboard.getInstance();
    private final Telemetry dashboardTelemetry = dashboard.getTelemetry();

    @Override
    public void initialize() {
        Drive drive = new Drive(hardwareMap, dashboardTelemetry);
        drive.setDefaultCommand(new TeleopDrive(drive, gamepad1));
        Arm arm = new Arm(hardwareMap, dashboardTelemetry);
        schedule(new RunCommand(dashboardTelemetry::update));

        Button a = new GamepadButton(new GamepadEx(gamepad1), GamepadKeys.Button.A);
        a.whenPressed(new ArmMovePosition(arm,1000));
        Button b = new GamepadButton(new GamepadEx(gamepad1), GamepadKeys.Button.B);
        b.whenPressed(new ArmMovePosition(arm,2000));

    }
}