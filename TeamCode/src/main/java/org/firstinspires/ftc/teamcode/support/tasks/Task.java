package org.firstinspires.ftc.teamcode.support.tasks;

/**
 * Represents a long running task that can be scheduled to execute
 *  in background without busy-waiting inside the event loop
 */
public abstract class Task {
    protected String name;

    /**
     * (Optional) Task name.
     * If specified, caller will be able to query <code>TaskManager</code>
     *  for this task's completion status
     */
    public String getName() {
        return name;
    }
    public void setName(String name) {
        this.name = name;
    }

    /**
     * Starts given task and returns its progress
     */
    public abstract Progress start();
}
