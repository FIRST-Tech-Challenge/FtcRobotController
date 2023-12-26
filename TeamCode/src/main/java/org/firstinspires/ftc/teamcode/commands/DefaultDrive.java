package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

import java.util.function.DoubleSupplier;

public class DefaultDrive extends CommandBase {
    private final DoubleSupplier forwardBackwardSupplier, leftRightSupplier, rotationSupplier;
    private final DriveSubsystem m_drive;


    public DefaultDrive(DriveSubsystem subsystem, DoubleSupplier forwardBackward,
                        DoubleSupplier leftRight, DoubleSupplier rotation) {
        m_drive = subsystem;
        forwardBackwardSupplier = forwardBackward;
        leftRightSupplier = leftRight;
        rotationSupplier = rotation;
        addRequirements(subsystem);
    }

    @Override
    public void execute() {
        m_drive.moveRobot(
                forwardBackwardSupplier.getAsDouble(),
                -leftRightSupplier.getAsDouble(),
                -rotationSupplier.getAsDouble()
        );
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
