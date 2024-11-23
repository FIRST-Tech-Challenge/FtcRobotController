package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.Climber;

public class ClimberUpCommand extends CommandBase {

    private Climber climber;

    public ClimberUpCommand(Climber climber) {
        this.climber = climber;
        addRequirements(climber);
    }

    @Override
    public void initialize() {
        climber.Active = true;
    }

    @Override
    public void execute() {
        climber.AddDegree();
    }


    @Override
    public boolean isFinished() {
        return true;
    }
}
