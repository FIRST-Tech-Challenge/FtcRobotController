package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.Climber;

public class ClimberDownCommand extends CommandBase {

    private Climber climber;

    public ClimberDownCommand(Climber climber) {
        this.climber = climber;
        addRequirements(climber);
    }

    @Override
    public void initialize() {
        climber.Active = true;
    }

    @Override
    public void execute() {
        climber.RemoveDegree();
    }


    @Override
    public boolean isFinished() {
        return true;
    }
}
