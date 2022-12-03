package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.dragonswpilib.command.CommandBase;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

public class TournerAutoCommande extends CommandBase {

    private final DriveSubsystem mDriveSubsystem;
    private final Telemetry mTelemetry;
    private int mAngle;
    private double mVitesse;


    public TournerAutoCommande(Telemetry telemetry, DriveSubsystem driveSubsystem,int angle, double vitesse){
        mTelemetry = telemetry;
        mDriveSubsystem = driveSubsystem;
        mAngle = angle;
        mVitesse = vitesse;
        addRequirements(driveSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        mDriveSubsystem.resetGyro();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        mDriveSubsystem.ArcadeDrive(0, mVitesse);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        mDriveSubsystem.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return Math.abs(mDriveSubsystem.getAngle()) >= mAngle;
    }
}
