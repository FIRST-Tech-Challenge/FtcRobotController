package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

public class TiltIntakeCommand extends CommandBase {

    IntakeSubsystem intakeSubsystem;
    boolean tilt;

    public TiltIntakeCommand(IntakeSubsystem intakeSubsystem, boolean tilt) {
        this.intakeSubsystem = intakeSubsystem;
        this.tilt = tilt;
        addRequirements(intakeSubsystem);
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void initialize() {
        if (tilt) {
            intakeSubsystem.tiltIntake();
        } else {
            intakeSubsystem.untiltIntake();
        }
    }
}
