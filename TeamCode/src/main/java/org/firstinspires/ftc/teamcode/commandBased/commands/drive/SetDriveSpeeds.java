package org.firstinspires.ftc.teamcode.commandBased.commands.drive;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.commandBased.subsystems.DrivetrainSubsystem;

public class SetDriveSpeeds extends CommandBase {

    private final DrivetrainSubsystem m_drivetrainSubsystem;
    private final double strafeMultiplier;
    private final double forwardMultiplier;
    private final double turnMultiplier;

    public SetDriveSpeeds(DrivetrainSubsystem drivetrainSubsystem,
                          double strafeMultiplier,
                          double forwardMultiplier,
                          double turnMultiplier) {
        m_drivetrainSubsystem = drivetrainSubsystem;
        addRequirements(m_drivetrainSubsystem);
        this.strafeMultiplier = strafeMultiplier;
        this.forwardMultiplier = forwardMultiplier;
        this.turnMultiplier = turnMultiplier;
    }

    @Override
    public void initialize() {
        m_drivetrainSubsystem.setSpeedMultipliers(
                strafeMultiplier,
                forwardMultiplier,
                turnMultiplier
        );
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
