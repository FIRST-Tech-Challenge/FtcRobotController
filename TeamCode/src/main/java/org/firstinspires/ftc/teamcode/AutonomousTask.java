package org.firstinspires.ftc.teamcode;

public abstract class AutonomousTask {
    protected Robot robot;

    // Constructor to initialize the task with the robot
    public AutonomousTask(Robot robot) {
        this.robot = robot;
    }

    // Abstract method to specify the task's logic
    public abstract void execute();

    // Abstract method to specify when the task is complete
    public abstract boolean isComplete();
}