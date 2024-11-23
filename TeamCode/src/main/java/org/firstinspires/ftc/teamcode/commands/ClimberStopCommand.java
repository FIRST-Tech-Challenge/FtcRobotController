package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.Climber;

public class ClimberStopCommand extends CommandBase {

    private Climber climber;

    public ClimberStopCommand(Climber climber) {
        this.climber = climber;
        addRequirements(climber);
    }

    @Override
    public void initialize() {
        climber.Active = false;
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
