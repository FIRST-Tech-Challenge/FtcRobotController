package org.firstinspires.ftc.teamcode.core.thread.old.types.api;

/**
 * Abstract class for a event that runs a listener a single time.
 */
public abstract class RunListenerOnceEvent extends EventAbstract {
    private final Runnable listener;

    /**
     * @param listener The listener to be run.
     */
    public RunListenerOnceEvent(Runnable listener) {
        this.listener = listener;
    }

    @Override
    public boolean run() {
        listener.run();
        return true;
    }
}
