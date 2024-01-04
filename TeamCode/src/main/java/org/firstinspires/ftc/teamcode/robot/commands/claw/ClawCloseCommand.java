package org.firstinspires.ftc.teamcode.robot.commands.claw;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.robot.subsystems.ClawSubsystem;

/**
 * open claw
 */
public class ClawCloseCommand extends CommandBase
{
    private final ClawSubsystem clawSubsystem;
    public ClawCloseCommand(ClawSubsystem subsystem)
    {
        clawSubsystem = subsystem;
        addRequirements(clawSubsystem);
    }

    @Override
    public void initialize() {
        clawSubsystem.close();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
