package org.firstinspires.ftc.teamcode.commands.DuckRoller;

import org.firstinspires.ftc.teamcode.subsystems.DucksSubsystem;
import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class IndexDuckCommand extends CommandBase {
    private final DucksSubsystem m_duckSpiner;
    private final int m_rotation;
    private final double m_power;

    public IndexDuckCommand(DucksSubsystem armSubsystem, int rotation , double power) {
        m_duckSpiner = armSubsystem;
        m_rotation = rotation;
        m_power = power;

        addRequirements(armSubsystem);
    }

    @Override
    public void initialize() {
        m_duckSpiner.setTargetPosition(m_duckSpiner.getCurrentPosition() + m_rotation);
        m_duckSpiner.setPower(m_power);
    }

    @Override
    public boolean isFinished() {
        return m_duckSpiner.isBusy();
    }
}
