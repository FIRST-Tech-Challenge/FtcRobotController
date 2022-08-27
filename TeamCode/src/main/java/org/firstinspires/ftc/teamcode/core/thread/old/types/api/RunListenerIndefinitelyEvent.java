package org.firstinspires.ftc.teamcode.core.thread.old.types.api;

/**
 * Abstract class for a event that runs a listener an indefinite amount of times.
 */
public abstract class RunListenerIndefinitelyEvent extends EventAbstract {
    private final Runnable listener;

    /**
     * @param listener The listener to be run.
     */
    public RunListenerIndefinitelyEvent(Runnable listener) {
        this.listener = listener;
    }

    @Override
    public boolean run() {
        listener.run();
        return false;
    }
}
