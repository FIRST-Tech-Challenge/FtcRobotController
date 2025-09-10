package org.firstinspires.ftc.teamcode.Commands;

import org.firstinspires.ftc.teamcode.Subsystems.Subsystem;

public interface Command {

    void start();

    void execute();

    void end();

    boolean isFinished();

    Subsystem getRequiredSubsystem();
}