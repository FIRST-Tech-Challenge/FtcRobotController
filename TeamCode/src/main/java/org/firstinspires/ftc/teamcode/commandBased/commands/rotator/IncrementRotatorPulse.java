package org.firstinspires.ftc.teamcode.commandBased.commands.rotator;

import com.qualcomm.robotcore.hardware.PwmControl;

import org.firstinspires.ftc.teamcode.commandBased.classes.triggers.TriggerCommandBase;
import org.firstinspires.ftc.teamcode.commandBased.subsystems.RotatorSubsystem;

public class IncrementRotatorPulse extends TriggerCommandBase {

    private double[] range;
    private final double increment;
    private final boolean usLower;
    private final RotatorSubsystem m_rotatorSubsystem;

    public IncrementRotatorPulse(
            RotatorSubsystem rotatorSubsystem,
            double increment,
            boolean usLower
    ) {
        m_rotatorSubsystem = rotatorSubsystem;
        addRequirements(rotatorSubsystem);
        range = m_rotatorSubsystem.getPWMRange();
        this.increment = increment;
        this.usLower = usLower;
    }

    @Override
    public void initialize() {
        range = m_rotatorSubsystem.getPWMRange();
        PwmControl.PwmRange newRange;

        if (usLower) {
            if (range[1] + increment < 500) {
                newRange = new PwmControl.PwmRange(500, range[2]);
            } else if (range[1] + increment >= range[2]) {
                newRange = new PwmControl.PwmRange(range[2] - 10, range[2]);
            } else {
                newRange = new PwmControl.PwmRange(range[1] + increment, range[2]);
            }
        } else {
            if (range[2] + increment > 2500) {
                newRange = new PwmControl.PwmRange(range[1], 2500);
            } else if (range[2] + increment <= range[1]) {
                newRange = new PwmControl.PwmRange(range[1], range[1] + 10);
            } else {
                newRange = new PwmControl.PwmRange(range[1], range[2] + increment);
            }
        }

        range = new double[]{newRange.usFrame, newRange.usPulseLower, newRange.usPulseUpper};
        m_rotatorSubsystem.setPWMRange(newRange);
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public boolean isTriggered() {
        return true;
    }
}
