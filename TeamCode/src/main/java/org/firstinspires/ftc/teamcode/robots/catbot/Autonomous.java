package org.firstinspires.ftc.teamcode.robots.catbot;

import java.util.LinkedList;
import java.util.Queue;

class Autonomous {
    Queue<Task> behaviors;
    Robot robot;
    public Autonomous(Robot robot)
    {
        this.behaviors = new LinkedList<Task>();
        this.robot = robot;
    }
    public boolean runBehaviors()
    {
        if(behaviors.size() > 0) {
            if (!behaviors.peek().run()) {
                behaviors.poll();
                robot.driveTrain.resetMotors();
            }
            return true;
        }
        return false;
    }
    public void add(Task task)
    {
        behaviors.offer(task);
    }
}
