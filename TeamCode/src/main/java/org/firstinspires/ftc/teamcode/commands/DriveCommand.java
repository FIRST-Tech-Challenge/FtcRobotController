package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

import java.util.function.DoubleSupplier;

public class DriveCommand extends CommandBase {
    private double forwardBackward, leftRight, rotation;
    private final DriveSubsystem m_drive;

    public DriveCommand(DriveSubsystem subsystem, double forwardBackward,
                        double leftRight, double rotation) {
        m_drive = subsystem;
        forwardBackward = forwardBackward;
        leftRight = leftRight;
        rotation = rotation;
        addRequirements(subsystem);
    }

    @Override
    public void execute() {
        m_drive.moveRobotMecanum(forwardBackward, leftRight, rotation);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
