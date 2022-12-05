package org.firstinspires.ftc.teamcode.robots.catbot;

import java.util.LinkedList;
import java.util.Queue;

class Autonomous {
    Queue<Task> behaviors = new LinkedList<Task>();
    public Autonomous(Queue<Task> behaviors)
    {
        this.behaviors=behaviors;
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
}
