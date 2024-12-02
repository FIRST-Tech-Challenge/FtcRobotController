package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

import java.util.function.DoubleSupplier;

public class DefaultDrive extends CommandBase {
    private DoubleSupplier forwardBackwardSupplier, leftRightSupplier, rotationSupplier;
    private final DriveSubsystem m_drive;
    private Gamepad gamepad1;

    @Deprecated
    public DefaultDrive(DriveSubsystem subsystem, DoubleSupplier forwardBackward,
                        DoubleSupplier leftRight, DoubleSupplier rotation) {
        m_drive = subsystem;
        forwardBackwardSupplier = forwardBackward;
        leftRightSupplier = leftRight;
        rotationSupplier = rotation;
        addRequirements(subsystem);
    }

    public DefaultDrive(DriveSubsystem subsystem, Gamepad gamepad) {
        m_drive = subsystem;
        addRequirements(subsystem);
        gamepad1 = gamepad;
    }

    @Override
    public void execute() {
        m_drive.moveRobot(gamepad1);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
