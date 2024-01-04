package org.firstinspires.ftc.teamcode.robot.commands.claw;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.robot.subsystems.ClawSubsystem;

/**
 * open claw
 */
public class ClawOpenCommand extends CommandBase
{
    private final ClawSubsystem clawSubsystem;
    public ClawOpenCommand(ClawSubsystem subsystem)
    {
        clawSubsystem = subsystem;
        addRequirements(clawSubsystem);
    }

    @Override
    public void initialize() {
        clawSubsystem.open();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
