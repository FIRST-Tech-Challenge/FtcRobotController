package org.firstinspires.ftc.teamcode.support.tasks;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;

public class TaskManager {

    private static List<Task> taskQueue = new ArrayList<>();
    private static Task currentTask;
    private static Progress currentTaskProgress;

    // Task Manager is a singleton and should not be instantiated directly
    private TaskManager() {
    }

    public static void add(Task task) {
        taskQueue.add(task);
    }

    public static void add(Task task, String name) {
        if (name==null) throw new NullPointerException("Task name is required");
        for (Iterator<Task> it = taskQueue.iterator(); it.hasNext(); ) {
            Task scheduledTask = it.next();
            if (scheduledTask.getName() != null && !scheduledTask.getName().equals(name)) {
                it.remove();
            }
        }
        if (currentTask!=null && currentTask.getName()!=null && !currentTask.getName().equals(name)) {
            currentTask = null;
            currentTaskProgress = null;
        }
        task.setName(name);
        taskQueue.add(task);
    }

    public static boolean isComplete(String taskName) {
        if (currentTask!=null && taskName.equals(currentTask.getName())) {
            return false;
        }
        for (Task task : taskQueue) {
            if (taskName.equals(task.getName())) return false;
        }
        return true;
    }

    public static boolean isEmpty() {
        if (currentTask!=null || taskQueue.isEmpty()==false) {
            return false;
        }
        return true;
    }

    public static void processTasks() {
        if (currentTaskProgress!=null) {
            // there's a task currently being executed
            if (!currentTaskProgress.isDone()) return;
            currentTask = null;
            currentTaskProgress = null;
        }
        // no more tasks to execute
        if (taskQueue.isEmpty()) return;
        currentTask = taskQueue.remove(0);
        currentTaskProgress = currentTask.start();
        // task did not return its completion progress and so is considered done
        if (currentTaskProgress==null) currentTask = null;
    }
}
