package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.Intake;

public class PivotIntake extends CommandBase {

    private Intake intake; 
    private Intake.IntakeState desiredState;
    
    public PivotIntake(Intake.IntakeState desired, Intake intake) {
        this.intake = intake;
        this.desiredState = desired;
    }
    
    @Override
    public void execute() {
        intake.setPivot(desiredState);
    }

    @Override
    public boolean isFinished() {
        return true; 
    }
}
