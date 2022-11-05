package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.dragonswpilib.CommandBase;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

public class DriveAutoCommand extends CommandBase {

    private final DriveSubsystem mDriveSubsystem;
    private final Telemetry mTelemetry;

    public DriveAutoCommand(Telemetry telemetry, DriveSubsystem driveSubsystem){
        mTelemetry = telemetry;
        mDriveSubsystem = driveSubsystem;

        addRequirements(driveSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        mDriveSubsystem.drive(0, 0.5, 0);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        mDriveSubsystem.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return true;
    }
}
