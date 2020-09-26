package org.firstinspires.ftc.teamcode.support.tasks;

/**
 * Will be returned from tasks that may take some time to complete.
 */
public interface Progress {

    /**
     * Indicates whether the task is completed
     */
    boolean isDone();
}
