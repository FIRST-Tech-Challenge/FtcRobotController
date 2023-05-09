package org.firstinspires.ftc.teamcode.commandBased.commands.drive;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.commandBased.subsystems.DrivetrainSubsystem;

public class FieldCentric extends CommandBase {

    private double leftStickX, leftStickY, rightStickX, turn;
    private DrivetrainSubsystem m_drivetrainSubsystem;
    //private LocalizerSubsystem m_localizerSubsystem;

    public FieldCentric(DrivetrainSubsystem drivetrainSubsystem, double leftStickX, double leftStickY, double rightStickX) {
        m_drivetrainSubsystem = drivetrainSubsystem;
        addRequirements(m_drivetrainSubsystem);
        this.leftStickX = leftStickX;
        this.leftStickY = leftStickY;
        this.rightStickX = rightStickX;
    }

    @Override
    public void initialize() {
        m_drivetrainSubsystem.setSpeedMultipliers(1, 1, 1);
    }

    @Override
    public void execute() {
        this.turn = m_drivetrainSubsystem.getTurnAmount(rightStickX);
        m_drivetrainSubsystem.fieldCentric(leftStickX, leftStickY, rightStickX);
    }
}
