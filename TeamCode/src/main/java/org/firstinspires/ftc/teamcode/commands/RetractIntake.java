package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.Intake;

public class RetractIntake extends CommandBase {
    private Intake intake;
    public RetractIntake(Intake intake) {
        this.intake = intake;
        addRequirements(intake);
    }

    public void execute() {

        intake.retract();

    }

    public boolean isFinished() {
        return true;
    }
}
