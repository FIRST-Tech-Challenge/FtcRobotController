package org.firstinspires.ftc.teamcode.tasks;

import java.util.ArrayList;
import java.util.List;

public abstract class Task implements Runnable {
    private TaskQueue _taskQueue;
    private String _name;

    public Task(TaskQueue taskQueue, String name, List<Task> dependsOn) {
        _taskQueue = taskQueue;
        _name = name;
        taskQueue.addTask(this, dependsOn);
    }
    public Task(TaskQueue taskQueue, String name)
    {
        _taskQueue = taskQueue;
        _name = name;
        taskQueue.addTask(this, new ArrayList<>());
    }
    /**
     * Start running the task.
     */
    public void run()
    {
        try{
            execute();
        }
        catch (Exception ex)
        {
            ex.printStackTrace();
        }
        _taskQueue.finishTask(this);

    }
    /**
     * Execute the task
     * Override this method to implement the task
     */
    protected abstract void execute();


}