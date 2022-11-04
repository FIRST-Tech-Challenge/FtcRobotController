package org.firstinspires.ftc.teamcode.dragonswpilib;

import org.firstinspires.ftc.teamcode.dragonswpilib.CommandBase;
import org.firstinspires.ftc.teamcode.dragonswpilib.Timer;

/**
 * A command that does nothing but takes a specified amount of time to finish. Useful for
 * CommandGroups. Can also be subclassed to make a command with an internal timer.
 *
 * <p>This class is provided by the NewCommands VendorDep
 */
public class WaitCommand extends CommandBase {
    protected Timer m_timer = new Timer();
    private final double m_duration;

    /**
     * Creates a new WaitCommand. This command will do nothing, and end after the specified duration.
     *
     * @param seconds the time to wait, in seconds
     */
    public WaitCommand(double seconds) {
        m_duration = seconds;
    }

    @Override
    public void initialize() {
        m_timer.reset();
        m_timer.start();
    }

    @Override
    public void end(boolean interrupted) {
        m_timer.stop();
    }

    @Override
    public boolean isFinished() {
        return m_timer.hasElapsed(m_duration);
    }
}