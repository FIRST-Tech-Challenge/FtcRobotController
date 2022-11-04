package org.firstinspires.ftc.teamcode.dragonswpilib;

/**
 * Class that holds scheduling state for a command. Used internally by the {@link CommandScheduler}.
 *
 * <p>This class is provided by the NewCommands VendorDep
 */
public class CommandState {
    // The time since this command was initialized.
    private double m_startTime = -1;

    // Whether or not it is interruptible.
    private final boolean m_interruptible;

    CommandState(boolean interruptible) {
        m_interruptible = interruptible;
        startTiming();
        startRunning();
    }

    private void startTiming() {
        //m_startTime = Timer.getFPGATimestamp();
    }

    synchronized void startRunning() {
        m_startTime = -1;
    }

    boolean isInterruptible() {
        return m_interruptible;
    }

    double timeSinceInitialized() {
        return -1;//m_startTime != -1 ? Timer.getFPGATimestamp() - m_startTime : -1;
    }
}
