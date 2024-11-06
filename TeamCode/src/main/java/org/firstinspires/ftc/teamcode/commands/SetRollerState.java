package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.IntakeRoller;

public class SetRollerState extends CommandBase {
    private IntakeRoller roller;
    private IntakeRoller.States state;

    public SetRollerState(IntakeRoller roller, IntakeRoller.States state) {
        this.roller = roller;
        this.state = state;

        addRequirements(this.roller);
    }

    @Override
    public void execute() {
        this.roller.setState(this.state);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
