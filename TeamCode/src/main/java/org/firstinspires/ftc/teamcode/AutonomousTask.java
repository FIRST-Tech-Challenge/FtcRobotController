package org.firstinspires.ftc.teamcode;

abstract class AutonomousTask {
    protected Robot robot;

    // Constructor to initialize the autonomous task with a robot
    public AutonomousTask(Robot robot) {
        this.robot = robot;
    }

    // Method to execute the task; to be implemented by subclasses
    public abstract void execute();

    // Method to check if the task is complete; to be implemented by subclasses
    public abstract boolean isComplete();
}