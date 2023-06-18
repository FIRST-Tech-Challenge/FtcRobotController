package org.firstinspires.ftc.teamcode.commandBased.commands.arm;

import org.firstinspires.ftc.teamcode.classes.triggers.TriggerCommandBase;
import org.firstinspires.ftc.teamcode.commandBased.Constants;
import org.firstinspires.ftc.teamcode.commandBased.subsystems.ArmSubsystem;

public class MoveArmToAngle extends TriggerCommandBase {

    private final double angle;
    private final double velo;
    private final double accel;
    private final ArmSubsystem m_armSubsystem;

    public MoveArmToAngle(ArmSubsystem armSubsystem, double angle, double velo, double accel) {
        m_armSubsystem = armSubsystem;
        addRequirements(m_armSubsystem);
        this.angle = angle;
        this.velo = velo;
        this.accel = accel;
    }

    @Override
    public void initialize() {
        m_armSubsystem.setArmAngle(angle, velo, accel);
    }

    @Override
    public boolean isFinished() {
        return m_armSubsystem.isFinished();
    }

    @Override
    public boolean isTriggered() {
        return (m_armSubsystem.getArmAngle() >= Constants.ARM_ANGLE_TRIGGER ||
                m_armSubsystem.getArmAngle() <= -Constants.ARM_ANGLE_TRIGGER);
    }
}
