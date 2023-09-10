package org.firstinspires.ftc.teamcode.robots.ri2d2023;

import java.util.LinkedList;
import java.util.Queue;

public class Autonomous {
    Queue<Task> behaviors;
    Robot robot;
//    int tag;
    public Autonomous(Robot robot)
    {
        behaviors = new LinkedList<Task>();
        this.robot = robot;
    }
    public boolean runBehaviors()
    {
//        robot.telemetry.addData("running to tag", )
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
    public boolean hasBehaviors()
    {
        return behaviors.size() > 0;
    }
    public void clearAuton () { behaviors.clear(); }
}
