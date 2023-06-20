package org.firstinspires.ftc.teamcode.commandBased.commands.arm;

import org.firstinspires.ftc.teamcode.classes.triggers.TriggerCommandBase;
import org.firstinspires.ftc.teamcode.commandBased.Constants;
import org.firstinspires.ftc.teamcode.commandBased.subsystems.ArmSubsystem;

public class MoveArmToAngle extends TriggerCommandBase {

    private final double angle;
    private final ArmSubsystem m_armSubsystem;

    public MoveArmToAngle(ArmSubsystem armSubsystem, double angle) {
        m_armSubsystem = armSubsystem;
        addRequirements(m_armSubsystem);
        this.angle = angle;
    }

    @Override
    public void initialize() {
        double velo = 10;
        double accel = 10;

        if (m_armSubsystem.getArmTargetAngle() == Constants.ARM_ANGLE_FRONT) {
            if (angle == Constants.ARM_ANGLE_IDLE) {
                velo = Constants.ARM_VELO_FRONT_IDLE;
                accel = Constants.ARM_ACCEL_FRONT_IDLE;
            } else if (angle == Constants.ARM_ANGLE_BACK) {
                velo = Constants.ARM_VELO_FRONT_BACK;
                accel = Constants.ARM_ACCEL_FRONT_BACK;
            }
        } else if (m_armSubsystem.getArmTargetAngle() == Constants.ARM_ANGLE_IDLE) {
            if (angle == Constants.ARM_ANGLE_FRONT) {
                velo = Constants.ARM_VELO_IDLE_FRONT;
                accel = Constants.ARM_ACCEL_IDLE_FRONT;
            } else if (angle == Constants.ARM_ANGLE_BACK) {
                velo = Constants.ARM_VELO_IDLE_BACK;
                accel = Constants.ARM_ACCEL_IDLE_BACK;
            }
        } else if (m_armSubsystem.getArmTargetAngle() == Constants.ARM_ANGLE_BACK) {
            if (angle == Constants.ARM_ANGLE_FRONT) {
                velo = Constants.ARM_VELO_FRONT_BACK;
                accel = Constants.ARM_ACCEL_FRONT_BACK;
            } else if (angle == Constants.ARM_ANGLE_IDLE) {
                velo = Constants.ARM_VELO_BACK_IDLE;
                accel = Constants.ARM_ACCEL_BACK_IDLE;
            }
        }

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
