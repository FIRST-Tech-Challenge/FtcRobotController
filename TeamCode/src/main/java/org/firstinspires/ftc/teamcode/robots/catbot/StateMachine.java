package org.firstinspires.ftc.teamcode.robots.catbot;

import java.util.LinkedList;
import java.util.Queue;

class StateMachine {
    Queue<Task> behaviors = new LinkedList<Task>();
    public StateMachine(Queue<Task> behaviors)
    {
        this.behaviors=behaviors;
    }
    public void runBehaviors()
    {
        if(behaviors.size() > 0)
            behaviors.peek().run();
    }
}
