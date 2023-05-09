package org.firstinspires.ftc.teamcode.commandBased.commands.drive;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.commandBased.subsystems.DrivetrainSubsystem;

public class RobotCentric extends CommandBase {

    private double leftStickX, leftStickY, rightStickX, turn;
    private DrivetrainSubsystem m_drivetrainSubsystem;

    public RobotCentric(DrivetrainSubsystem drivetrainSubsystem, double leftStickX, double leftStickY, double rightStickX) {
        m_drivetrainSubsystem = drivetrainSubsystem;
        addRequirements(m_drivetrainSubsystem);
        this.leftStickX = leftStickX;
        this.leftStickY = leftStickY;
        this.rightStickX = rightStickX;
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {

    }
}
