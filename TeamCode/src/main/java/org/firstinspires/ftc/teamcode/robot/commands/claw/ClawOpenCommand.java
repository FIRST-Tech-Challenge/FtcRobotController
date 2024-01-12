package org.firstinspires.ftc.teamcode.robot.commands.claw;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.robot.subsystems.ClawSubsystem;

/**
 * open claw
 */
public class ClawOpenCommand extends CommandBase
{
    public enum Side {
        BOTH,
        LEFT,
        RIGHT
    }
    private final ClawSubsystem clawSubsystem;
    private final Side type;
    public ClawOpenCommand(ClawSubsystem subsystem, Side type)
    {
        this.type = type;
        clawSubsystem = subsystem;
        addRequirements(clawSubsystem);
    }

    @Override
    public void initialize() {
        switch (type) {
            case BOTH:
                clawSubsystem.openBoth();
                break;
            case LEFT:
                clawSubsystem.openLeft();
                break;
            case RIGHT:
                clawSubsystem.openRight();
                break;
        }
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
