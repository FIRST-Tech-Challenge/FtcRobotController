package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.Intake;

public class SetKickerPosition extends CommandBase {

    private Intake intake;
    private boolean retract;

    public SetKickerPosition(boolean retract, Intake intake) {
        this.intake = intake;
        this.retract = retract;

        addRequirements(intake);
    }

    @Override
    public void execute() {
        if (retract) {
            intake.retractKicker();
        } else {
            intake.extendKicker();
        }
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
