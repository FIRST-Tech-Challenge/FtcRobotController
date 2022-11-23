package org.firstinspires.ftc.teamcode.commands;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.dragonswpilib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

public class DriveCommand extends CommandBase {

    private final DriveSubsystem mDriveSubsystem;
    private final Telemetry mTelemetry;
    private final Gamepad mGamepad;

    public DriveCommand(Telemetry telemetry, DriveSubsystem driveSubsystem, Gamepad gamepad){
        mTelemetry = telemetry;
        mDriveSubsystem = driveSubsystem;
        mGamepad = gamepad;

        addRequirements(driveSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double axeX = mGamepad.left_stick_x;
        double axeY = -mGamepad.left_stick_y;
        double axeZ = mGamepad.right_stick_x;
        mDriveSubsystem.drive(axeX, axeY, axeZ);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        mDriveSubsystem.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
