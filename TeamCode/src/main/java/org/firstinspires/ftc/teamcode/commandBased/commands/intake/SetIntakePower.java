package org.firstinspires.ftc.teamcode.commandBased.commands.intake;

import org.firstinspires.ftc.teamcode.commandBased.classes.triggers.TriggerCommandBase;
import org.firstinspires.ftc.teamcode.commandBased.Constants;
import org.firstinspires.ftc.teamcode.commandBased.subsystems.IntakeSubsystem;

public class SetIntakePower extends TriggerCommandBase {

    private final IntakeSubsystem m_intakeSubsystem;
    private final double power;

    public SetIntakePower(IntakeSubsystem intakeSubsystem, double power) {
        m_intakeSubsystem = intakeSubsystem;
        addRequirements(m_intakeSubsystem);
        this.power = power;
    }

    @Override
    public void initialize() {
        m_intakeSubsystem.setPower(power);
    }

    @Override
    public boolean isFinished() {
        if (power == Constants.INTAKE_IN) {
            return (m_intakeSubsystem.getAverageCurrent() > 650);
        } else if (power == Constants.INTAKE_OUT){
            return (m_intakeSubsystem.getAverageCurrent() > 150);
        } else {
            return true;
        }
    }

    @Override
    public boolean isTriggered() {
        return true;
    }
}
