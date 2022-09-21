package org.firstinspires.ftc.teamcode.opmodes.createmechanism;

import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commands.drive.bc4h.DefaultDrive;
import org.firstinspires.ftc.teamcode.commands.drive.bc4h.ToggleSlowdown;
import org.firstinspires.ftc.teamcode.commands.intake.MoveIntake;
import org.firstinspires.ftc.teamcode.subsystems.drive.bc4h.BC4HDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeSubsystem;

public class CreateBC4HDrive {

    private BC4HDriveSubsystem drive;
    private final HardwareMap hwMap;
    private final String deviceNameFl;
    private final String deviceNameFr;
    private final String deviceNameBl;
    private final String deviceNameBr;
    private final Telemetry telemetry;
    private final GamepadEx op;

    private static final int REV_ENCODER_CLICKS = 537;
    private static final double REV_WHEEL_DIAM = 7.5;

    public CreateBC4HDrive(final HardwareMap hwMap,
                           final String deviceNameFl,
                           final String deviceNameFr,
                           final String deviceNameBl,
                           final String deviceNameBr, final GamepadEx op, Telemetry telemetry){

        this.deviceNameFl = deviceNameFl;
        this.deviceNameFr = deviceNameFr;
        this.deviceNameBl = deviceNameBl;
        this.deviceNameBr = deviceNameBr;

        this.hwMap = hwMap;
        this.op = op;
        this.telemetry = telemetry;

    }

    public CreateBC4HDrive(final HardwareMap hwMap,
                           final String deviceNameFl,
                           final String deviceNameFr,
                           final String deviceNameBl,
                           final String deviceNameBr, final GamepadEx op, Telemetry telemetry, boolean autoCreate){

        this.deviceNameFl = deviceNameFl;
        this.deviceNameFr = deviceNameFr;
        this.deviceNameBl = deviceNameBl;
        this.deviceNameBr = deviceNameBr;

        this.hwMap = hwMap;
        this.op = op;
        this.telemetry = telemetry;

        if (autoCreate) create();

    }

    public void create(){


        drive = new BC4HDriveSubsystem(hwMap,
                deviceNameFl,
                deviceNameFr,
                deviceNameBl,
                deviceNameBr,
                REV_ENCODER_CLICKS, REV_WHEEL_DIAM,
                DcMotorEx.Direction.FORWARD,
                DcMotorEx.Direction.REVERSE,
                DcMotorEx.Direction.FORWARD,
                DcMotorEx.Direction.REVERSE,
                DcMotorEx.RunMode.RUN_USING_ENCODER );

        Button slowDown = new GamepadButton(op, GamepadKeys.Button.RIGHT_BUMPER);
        ToggleSlowdown toggleSlowdown = new ToggleSlowdown(drive, telemetry);
        slowDown.whenPressed(toggleSlowdown);

        DefaultDrive driveRobot = new DefaultDrive(drive,
                () -> op.getLeftX(),
                () -> op.getLeftY(),
                () -> op.getRightX());
        drive.setDefaultCommand(driveRobot);
    }
}
