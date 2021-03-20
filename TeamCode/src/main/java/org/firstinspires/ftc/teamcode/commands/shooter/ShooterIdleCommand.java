package org.firstinspires.ftc.teamcode.commands.shooter;

import com.technototes.library.command.Command;

import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;

public class ShooterIdleCommand extends Command {
    public ShooterSubsystem subsystem;
    public ShooterIdleCommand(ShooterSubsystem s){
        addRequirements(s);
        subsystem = s;
    }

    @Override
    public void init() {
        subsystem.setVelocity(subsystem.getIdleVelocity());
    }

    @Override
    public boolean isFinished() {
        return subsystem.isAtIdleVelocity();
    }
}
