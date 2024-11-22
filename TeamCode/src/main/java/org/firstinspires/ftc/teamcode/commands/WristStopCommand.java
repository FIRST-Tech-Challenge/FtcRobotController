package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.Wrist;

public class WristStopCommand extends CommandBase {

    private Wrist wrist;

    public WristStopCommand(Wrist wrist) {
        this.wrist = wrist;
        addRequirements(wrist);
    }

    @Override
    public void initialize() {
        wrist.Active = false;
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
