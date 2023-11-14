package org.firstinspires.ftc.teamcode.tasks.samples;

import org.firstinspires.ftc.teamcode.RobotConfig;
import org.firstinspires.ftc.teamcode.tasks.Task;
import org.firstinspires.ftc.teamcode.tasks.TaskQueue;

import java.util.List;

public class MoveTask extends Task {

    public MoveTask(TaskQueue taskQueue, String name, List<Task> dependsOn) {
        super(taskQueue, name, dependsOn);
    }

    public MoveTask(TaskQueue taskQueue, String name) {
        super(taskQueue, name);
    }

    @Override
    protected void execute() {
        RobotConfig.getInstance().x = 2;
    }
}
