package org.firstinspires.ftc.teamcode.Components;

import static org.firstinspires.ftc.teamcode.BasicRobot.op;

import java.util.ArrayList;

public class Queuer {
    private ArrayList<QueueElement> queueElements;
    private boolean firstLoop = true;
    private int currentlyQueueing = 0, currentEvent = -1;


    public Queuer() {
        queueElements = new ArrayList<QueueElement>();
    }

    public void setFirstLoop(boolean p_firstLoop) {
        firstLoop = p_firstLoop;
    }

    public boolean queue(boolean p_asynchronous, boolean done_condition) {
        op.telemetry.addData("currentEvent", currentEvent);
        op.telemetry.addData("currentlyQueueing", currentlyQueueing);
        for (int i = 0; i < queueElements.size(); i++) {
            op.telemetry.addData("startCon" + i, queueElements.get(i).startCondition);
            op.telemetry.addData("isStarted" + i, queueElements.get(i).isStarted());
            op.telemetry.addData("done" + i, queueElements.get(i).isDone());
            op.telemetry.addData("isReady" + i, queueElements.get(i).isReady(currentEvent));
        }
        op.telemetry.update();
        if (firstLoop) {
            int startCondition = -1;
            for (int i = 0; i < queueElements.size(); i++) {
                if (!queueElements.get(queueElements.size() - i - 1).isAsynchronous()) {
                    startCondition = queueElements.size() - i - 1;
                    break;
                }
            }
            queueElements.add(new QueueElement(queueElements.size(), p_asynchronous, startCondition));
        }
        if (currentlyQueueing >= queueElements.size() - 1) {
            currentlyQueueing = 0;
        } else {
            currentlyQueueing++;
        }
        if (queueElements.get(currentlyQueueing).isStarted() && !queueElements.get(currentlyQueueing).isDone()) {
            queueElements.get(currentlyQueueing).setDone(done_condition);
            if (done_condition) {
                currentEvent = currentlyQueueing;
            }
        }
        boolean isReady = queueElements.get(currentlyQueueing).isReady(currentEvent);
        if (isReady) {
            queueElements.get(currentlyQueueing).setStarted(true);
        }
        return isReady;
    }
}