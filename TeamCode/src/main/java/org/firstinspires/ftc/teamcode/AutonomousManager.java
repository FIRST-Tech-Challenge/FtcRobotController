package org.firstinspires.ftc.teamcode;

import java.util.LinkedList;
import java.util.Queue;

class AutonomousManager {
    private Queue<AutonomousTask> tasks; // Queue to manage autonomous tasks
    private AutonomousTask currentTask; // Current task being executed

    // Constructor to initialize the task manager
    public AutonomousManager() {
        tasks = new LinkedList<>();
        System.out.println("AutonomousManager initialized");
    }

    // Add a new task to the queue
    public void addTask(AutonomousTask task) {
        tasks.add(task);
        System.out.println("Task added to queue");
    }

    // Update method to execute the current task or move to the next task
    public void update() {
        // If there is no current task or it is complete, move to the next task in the queue
        if (currentTask == null || currentTask.isComplete()) {
            if (!tasks.isEmpty()) {
                currentTask = tasks.poll(); // Get the next task from the queue
                System.out.println("New task started");
            } else {
                currentTask = null; // No more tasks to execute
                System.out.println("No more tasks to execute");
            }
        }

        // If there is a current task, execute it
        if (currentTask != null) {
            currentTask.execute();
        }
    }
}