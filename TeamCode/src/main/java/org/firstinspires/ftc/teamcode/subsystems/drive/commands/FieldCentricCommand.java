package org.firstinspires.ftc.teamcode.subsystems.drive.commands;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.drive.MecanumDriveSubsystem;

import java.util.function.DoubleSupplier;

public final class FieldCentricCommand extends CommandBase {
    private final MecanumDriveSubsystem driveSubsystem;
    private final DoubleSupplier driveSupplier, strafeSupplier, turnSupplier;

    /**
     * Drives the robot using the provided suppliers. Note that this function only ends when
     * interrupted
     * @param driveSubsystem The drive subsystem to move
     * @param driveSupplier The function which supplies the drive value to the command, typically
     *                      a method reference, or lambda expression.
     * @param strafeSupplier The function which supplies the strafe value to the command, typically
     *                       a method reference, or lambda expression.
     * @param turnSupplier The function which supplies the turn value to the command, typically a
     *                     method reference, or lambda expression.
     */
    public FieldCentricCommand(
            @NonNull MecanumDriveSubsystem driveSubsystem,
            @NonNull DoubleSupplier driveSupplier,
            @NonNull DoubleSupplier strafeSupplier,
            @NonNull DoubleSupplier turnSupplier
    ) {
        this.driveSubsystem = driveSubsystem;
        this.driveSupplier  = driveSupplier;
        this.strafeSupplier = strafeSupplier;
        this.turnSupplier   = turnSupplier;

        addRequirements(driveSubsystem);
    }

    @Override public void execute() {
        driveSubsystem.driveFieldCentric(
                driveSupplier.getAsDouble(),
                strafeSupplier.getAsDouble(),
                turnSupplier.getAsDouble()
        );
    }
}