package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.Intake;

public class ExtendIntake extends CommandBase {
    private Intake intake;

    public ExtendIntake(Intake intake) {
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void execute() {
        this.intake.extend();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
