package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

public class IntakeOut extends CommandBase {
    private Intake m_intake;
    private Telemetry m_telemetry;


    public IntakeOut(Intake intake, Telemetry telemetry) {
        m_intake = intake;
        m_telemetry = telemetry;
        addRequirements(intake);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        m_intake.runIntake(-1.0);
    }

    @Override
    public boolean isFinished() {
        return true;
    }


    public void end() {
        m_intake.runIntake(0.0);
    }
}
