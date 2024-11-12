package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.FingerSubsystem;

public class MoveFingerCommand extends CommandBase {

    FingerSubsystem subsystem;
    FingerSubsystem.FingerPositions position;
    boolean ran = false;

    public MoveFingerCommand(FingerSubsystem subsystem, FingerSubsystem.FingerPositions position) {
        this.subsystem = subsystem;
        this.position = position;
        addRequirements(subsystem);
    }

    @Override
    public void execute() {
        subsystem.locomoteFinger(position);
        ran = true;
    }

    @Override
    public boolean isFinished() {
        return ran;
    }
}
