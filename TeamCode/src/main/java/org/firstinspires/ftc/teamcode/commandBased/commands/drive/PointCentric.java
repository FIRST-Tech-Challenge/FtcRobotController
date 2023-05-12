package org.firstinspires.ftc.teamcode.commandBased.commands.drive;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.classes.Pose2d;
import org.firstinspires.ftc.teamcode.classes.Vector2d;
import org.firstinspires.ftc.teamcode.commandBased.subsystems.DrivetrainSubsystem;

import java.util.function.DoubleSupplier;

public class PointCentric extends CommandBase {

    private final DoubleSupplier strafeSpeed;
    private final DoubleSupplier forwardSpeed;
    private final DrivetrainSubsystem m_drivetrainSubsystem;
    private final Vector2d target;
    private final Pose2d pose;

    public PointCentric(DrivetrainSubsystem drivetrainSubsystem,
                        DoubleSupplier strafeSpeed,
                        DoubleSupplier forwardSpeed,
                        Vector2d target,
                        Pose2d pose)
    {
        m_drivetrainSubsystem = drivetrainSubsystem;
        addRequirements(m_drivetrainSubsystem);
        this.strafeSpeed = strafeSpeed;
        this.forwardSpeed = forwardSpeed;
        this.target = target;
        this.pose = pose;
    }

    @Override
    public void execute() {
        m_drivetrainSubsystem.pointCentric(
                strafeSpeed.getAsDouble(),
                forwardSpeed.getAsDouble(),
                target,
                pose
        );
    }
}
