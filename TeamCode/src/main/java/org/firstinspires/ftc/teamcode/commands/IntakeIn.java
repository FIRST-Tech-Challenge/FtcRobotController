package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

public class IntakeIn extends CommandBase {
    private Intake m_intake;
    private Telemetry m_telemetry;

    public IntakeIn(Intake intake, Telemetry telemetry) {
        m_intake = intake;
        m_telemetry = telemetry;
        addRequirements(m_intake);
    }

    @Override
    public void initialize() {
        m_telemetry.addData("Subsystem IntakeIn Command", "Initialized");

    }

    @Override
    public void execute() {
        m_intake.runIntake(1.0);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
