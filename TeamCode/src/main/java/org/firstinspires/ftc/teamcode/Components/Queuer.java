package org.firstinspires.ftc.teamcode.Components;

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

    /** creates new queue element if first loop
    * updates which element is currently being queued and which element is currently being executed
    * determines if currently queued element should run */
    public boolean queue(boolean p_asynchronous, boolean done_condition) {
        //create new queue element if it is first loop
        if (firstLoop) {
            createQueueElement(p_asynchronous);
        }

        //update which element is currently being queued & which event is currently being executed
        updateQueuer(done_condition);

        //determine if currently queued element should be executed
        boolean isReady = queueElements.get(currentlyQueueing).isReady(currentEvent);

        //set queueElement internal value
        if (isReady) {
            queueElements.get(currentlyQueueing).setStarted(true);
        }

        return isReady;
    }

    /** same as regular queue, but will wait inputted delay time before running */
    public boolean queue(boolean p_asyncrhonous, boolean done_condition, double p_delay) {
        //create new queue element if it is first loop, with delay instead of asynchronous
        if (firstLoop) {
            createQueueElement(p_asyncrhonous,p_delay);
        }

        //update which element is currently being queued & which event is currently being executed
        updateQueuer(done_condition);

        //determine if currently queued element should be executed
        boolean isReady = queueElements.get(currentlyQueueing).isReady(currentEvent);

        //set queueElement internal value
        if (isReady) {
            queueElements.get(currentlyQueueing).setStarted(true);
        }

        return isReady;
    }

    /** same as regular queue, but will wait for extra_condition to be true before running */
    public boolean queue(boolean p_asynchronous, boolean done_condition, boolean extra_condition) {
        //create new queue element if it is first loop
        if (firstLoop) {
            createQueueElement(p_asynchronous);
        }

        //update which element is currently being queued & which event is currently being executed
        updateQueuer(done_condition);

        //determine if currently queued element should be executed with extra_condition(probably position threshold)
        boolean isReady = queueElements.get(currentlyQueueing).isReady(currentEvent, extra_condition);

        //set queueElement internal value
        if (isReady) {
            queueElements.get(currentlyQueueing).setStarted(true);
        }
        return isReady;
    }

    public void resetQueuer() {
        queueElements.clear();
    }

    public void reset() {
        queueElements.clear();
    }
    /** create new queueElement*/
    private void createQueueElement(boolean p_asynchronous) {
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
    }
    /** create new queueElement with extra delay*/
    private void createQueueElement(boolean p_asynchrnous, double p_delay) {
        int startCondition = -1;
        for (int i = 0; i < queueElements.size(); i++) {
            if (!queueElements.get(queueElements.size() - i - 1).isAsynchronous()) {
                startCondition = queueElements.size() - i - 1;
                break;
            }
        }
        queueElements.add(new QueueElement(queueElements.size(), p_asynchrnous, startCondition, p_delay));
    }
    /** update which element is currently being queued(processed) and which element is currently being executed*/
    private void updateQueuer(boolean done_condition) {
        //update which element is currently being queued
        if (currentlyQueueing >= queueElements.size() - 1) {
            currentlyQueueing = 0;
        } else {
            currentlyQueueing++;
        }
        //update which event is currently being executed
        if (!firstLoop) {
            if (queueElements.get(currentlyQueueing).isStarted() && !queueElements.get(currentlyQueueing).isDone()) {
                queueElements.get(currentlyQueueing).setDone(done_condition);
                if (done_condition) {
                    currentEvent = currentlyQueueing;
                }
            }
        }
    }

}