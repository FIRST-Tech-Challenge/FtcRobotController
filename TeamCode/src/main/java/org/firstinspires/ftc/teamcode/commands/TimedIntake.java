package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.rustlib.commandsystem.WaitCommand;

public class TimedIntake extends WaitCommand {
    private final Intake subsystem;
    private final double speed;

    public TimedIntake(Intake subsystem, double speed, double timeMilliseconds) {
        super(timeMilliseconds);
        this.subsystem = subsystem;
        this.speed = speed;
    }

    @Override
    public void initialize() {
        super.initialize();
        subsystem.run(speed);
    }

    @Override
    public void end(boolean interrupted) {
        subsystem.run(0);
    }
}
