package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.GateSubsystem;

public class GateCommand extends CommandBase {

    GateSubsystem.GateState state;
    GateSubsystem subsystem;
    boolean finished = false;

    public GateCommand(GateSubsystem subsystem, GateSubsystem.GateState state) {
        this.subsystem = subsystem;
        this.state = state;
        addRequirements(subsystem);
    }

    @Override
    public void execute() {
        subsystem.setGateState(state);
        finished = true;
    }

    @Override
    public boolean isFinished() {
        return finished;
    }
}
