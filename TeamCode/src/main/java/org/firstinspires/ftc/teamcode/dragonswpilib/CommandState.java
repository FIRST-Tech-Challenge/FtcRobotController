package org.firstinspires.ftc.teamcode.dragonswpilib;

/**
 * Class that holds scheduling state for a command. Used internally by the {@link CommandScheduler}.
 *
 * <p>This class is provided by the NewCommands VendorDep
 */
public class CommandState {

    // Whether or not it is interruptible.
    private final boolean m_interruptible;

    CommandState(boolean interruptible) {
        m_interruptible = interruptible;
        startTiming();
        startRunning();
    }

    private void startTiming() {
    }

    synchronized void startRunning() {
    }

    boolean isInterruptible() {
        return m_interruptible;
    }

    double timeSinceInitialized() {
        return -1;
    }
}
