package org.firstinspires.ftc.teamcode.tasks;

import org.firstinspires.ftc.teamcode.RobotConfig;

import java.util.ArrayList;
import java.util.List;

public class TaskQueue {
    private static class TaskQueueItem {
        public Task Task;
        public List<Task> DependsOn;
        public Integer inGrade;
        public Thread Thread;
        public TaskQueueItem(Task task, List<Task> dependsOn) {
            Task = task;
            DependsOn = dependsOn;
            inGrade = dependsOn.size();
        }
    }
    public boolean finished = false;
    int _taskFinished = 0;
    List<TaskQueueItem> tasks;

    public TaskQueue() {
        tasks = new ArrayList<TaskQueueItem>();
    }

    /**
     * Start the task queue with the tasks with no dependencies
     */
    public void start() {
        RobotConfig.getInstance().init();
        for (int i = 0; i < tasks.size(); i++) {
            if (tasks.get(i).inGrade == 0) {
                tasks.get(i).Thread = new Thread(tasks.get(i).Task);
                tasks.get(i).Thread.start();
            }
        }
    }

    /**
     * Add a task to the queue
     * @param task The task to add
     * @param dependsOn The tasks that must be finished before this task can start
     */
    public void addTask(Task task, List<Task> dependsOn) {
        tasks.add(new TaskQueueItem(task,dependsOn));
    }

    /**
     * Finish a task and allow the tasks that depend on it to start
     * @param task The task that has finished
     */
    public synchronized void finishTask(Task task) {
        _taskFinished++;
        for (int i = 0; i < tasks.size(); i++) {
            if (tasks.get(i).DependsOn.contains(task)) {
                tasks.get(i).DependsOn.remove(task);
                tasks.get(i).inGrade--;
                if (tasks.get(i).inGrade == 0) {
                    tasks.get(i).Thread = new Thread(tasks.get(i).Task);
                    tasks.get(i).Thread.start();
                }
            }
        }
        _taskFinished++;
        if (_taskFinished == tasks.size())
            finished = true;
    }
}