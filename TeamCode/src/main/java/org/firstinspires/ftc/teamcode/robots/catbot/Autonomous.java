package org.firstinspires.ftc.teamcode.robots.catbot;

import java.util.LinkedList;
import java.util.Queue;

class Autonomous {
    Queue<Task> behaviors;
    public Autonomous()
    {
        this.behaviors = new LinkedList<Task>();
    }
    public boolean runBehaviors()
    {
        if(behaviors.size() > 0) {
            if (!behaviors.peek().run())
                behaviors.poll();
            return true;
        }
        return false;
    }
    public void add(Task task)
    {
        behaviors.offer(task);
    }
}
