package org.firstinspires.ftc.teamcode.Components.RFModules.System;

import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.LOGGER;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.logger;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.time;

import java.util.ArrayList;

/**
 * Warren Zhou
 * 8/23
 * function scheduler that allows for sequential, asynchronous, and other forms of scheduling
 */
public class Queuer {
    private final ArrayList<QueueElement> queueElements;
    private boolean firstLoop = true, mustFinish = false;
    private int currentlyQueueing = 0, currentEvent = -1, mustStartCondition = -1, completeCurrentEvent = 0;
    private double delay = 0;

    private boolean isFulfilled = false;


    public Queuer() {
        queueElements = new ArrayList<>();
    }

    public void setFirstLoop(boolean p_firstLoop) {
        firstLoop = p_firstLoop;
    }

    /**
     * creates new queue element if first loop
     * updates which element is currently being queued and which element is currently being executed
     * determines if currently queued element should run
     *
     * @param p_asynchronous   is the function asynchronous
     * @param p_done_condition is the function done
     * @return if the function should run
     */
    public boolean queue(boolean p_asynchronous, boolean p_done_condition) {
        double p_delay = delay;
        delay = 0;
        return queue(p_asynchronous, p_done_condition, p_delay);
    }

    /**
     * same as regular queue, but will have an extra condition
     *
     * @param p_asynchronous   is the function asynchronous
     * @param p_done_condition is the function done
     * @param p_isOptional     is the function optional
     * @return if the function should run
     */
    public boolean queue(boolean p_asynchronous, boolean p_done_condition, boolean p_isOptional) {
        double p_delay = delay;
        delay = 0;
        return queue(p_asynchronous, p_done_condition, true, p_isOptional);
    }

    /**
     * same as regular queue, but will wait inputted delay time before running
     *
     * @param p_asyncrhonous   is the function asynchronous
     * @param p_done_condition is the function done
     * @param p_delay          how much the function
     * @return if the function should run
     */
    public boolean queue(boolean p_asyncrhonous, boolean p_done_condition, double p_delay) {
        if (!firstLoop && currentlyQueueing >= queueElements.size() - 1) {
            currentlyQueueing = -1;
        }
        return queue(p_asyncrhonous, p_done_condition, !firstLoop && time -
                queueElements.get(currentlyQueueing + 1).getReadyTime() > p_delay, false);
    }

    /**
     * Set currently queued synchronous event to done, manually skipping it, overriding any other conditions
     * logs which  event gets finished at general surface level
     */
    public void done() {
        int inde = 908;
        for (int i = currentEvent + 1; i < queueElements.size(); i++) {
            if (!queueElements.get(i).isAsynchronous()) {
                inde = i;
                break;
            } else {
                queueElements.get(i).setDone(true);
            }
        }
        if (inde != 908) {
            queueElements.get(inde).setDone(true);
            calculateCompleteCurrentEvent();
            currentEvent = inde;
            logger.log("/RobotLogs/GeneralRobot", "currentEventDone" + currentEvent);
        }
    }

    /**
     * update start conditions of subsequent events after one is suddenly changed to non-optional
     * logs which  event gets updated at general fine level
     * @param p_ind index of queueElement in question
     */
    public void updateStartConditions(int p_ind) {
        if (p_ind < queueElements.size()) {
            if (queueElements.get(p_ind).startCondition != recalcStartPosSkipOptional(p_ind, queueElements.get(p_ind).isAsynchronous(), queueElements.get(p_ind).isOptional())) {
                queueElements.get(p_ind).setStartCondition(recalcStartPosSkipOptional(p_ind, queueElements.get(p_ind).isAsynchronous(), queueElements.get(p_ind).isOptional()));
                LOGGER.setLogLevel(RFLogger.Severity.FINE);
                LOGGER.log( "Event: "+ p_ind +" StartCondition: " + queueElements.get(p_ind).startCondition);
                updateStartConditions(p_ind + 1);
                updateStartConditions(p_ind + 2);
                updateStartConditions(p_ind + 3);

            }
        }
    }

    /**
     * return if the currently queued event +1 is started, useful as a time saving quick done condition
     */
    public boolean isStarted() {
        if (currentlyQueueing + 1 < queueElements.size()) {
            return queueElements.get(currentlyQueueing + 1).isStarted() || queueElements.get(currentlyQueueing + 1).isDone();
        } else if (currentlyQueueing + 1 == queueElements.size() && queueElements.size() != 0) {
            return queueElements.get(0).isStarted() || queueElements.get(0).isDone();
        }
        return false;
    }
    public boolean isExecuted(){
        return queueElements.get(currentlyQueueing).isExecuted();
    }

    /**
     * updates and processes all things related to currently queued event
     *
     * @param p_asynchronous    is the function asynchronous
     * @param p_done_condition  is the function done
     * @param p_extra_condition extra condition if the function should run
     * @param p_isOptional      is the function optional
     */
    public boolean queue(boolean p_asynchronous, boolean p_done_condition, boolean p_extra_condition, boolean p_isOptional) {
        boolean isStart = isStarted();
        p_done_condition = isStart&&p_done_condition;
        //if it is first Loop
        if (firstLoop) {
            //create queue element
            createQueueElement(p_asynchronous, p_isOptional);
        }

        //update which element is currently being queued & which event is currently being executed
        updateQueuer(p_done_condition, p_isOptional);
        if(isStart){
            queueElements.get(currentlyQueueing).setExecuted(true);
        }
        //save some processing time if the event is done alrdy
        if (queueElements.get(currentlyQueueing).isDone()) {
            return false;
        }
        //save some processing time if event is too far in future
        if (currentEvent < queueElements.get(currentlyQueueing).startCondition - 2) {
            return false;
        }
        boolean isReady;
        //is currently queued event optional and should be running for first time
        if (queueElements.get(currentlyQueueing).isOptional() && p_isOptional && !queueElements.get(currentlyQueueing).isDone() && !firstLoop) {
            //make this optional event have to run
            queueElements.get(currentlyQueueing).setSkipOption(true);
            //update start conditions of subsequent actions
            updateStartConditions(currentlyQueueing);
            //calculate if event should run
            isReady = queueElements.get(currentlyQueueing).isReady(currentEvent, p_extra_condition);
            if(currentlyQueueing>0)
                isReady = isReady&&(!queueElements.get(currentlyQueueing-1).isMustFinish()||queueElements.get(currentlyQueueing-1).isDone());
        }
        //if current event has normal start condition
        else if (!queueElements.get(currentlyQueueing).isMustFinish() && !queueElements.get(currentlyQueueing).isShouldFinish()) {
            //calculate if event should run
            isReady = queueElements.get(currentlyQueueing).isReady(currentEvent, p_extra_condition);
            if(currentlyQueueing>0)
                isReady = isReady&&(!queueElements.get(currentlyQueueing-1).isMustFinish()||queueElements.get(currentlyQueueing-1).isDone());
        }
        //if current event must wait for all previous events to finsih
        else {
            //calculate if event should run
            isReady = queueElements.get(currentlyQueueing).isReady(completeCurrentEvent, p_extra_condition);
            if(currentlyQueueing>0)
                isReady = isReady&&(!queueElements.get(currentlyQueueing-1).isMustFinish()||queueElements.get(currentlyQueueing-1).isDone());
        }

        //set queueElement internal value
        if (isReady) {
            //skip if event is optional
            if (queueElements.get(currentlyQueueing).isOptional() && !p_isOptional) {
                isReady = false;
            }
            //set queueElement internal value
            else {
                queueElements.get(currentlyQueueing).setStarted(true);
            }
        }
        //return value
        return isReady || queueElements.get(currentlyQueueing).isStarted() && !queueElements.get(currentlyQueueing).isDone();
    }

    /**
     * set the current event to whatever is currently being queued, skip back or forward into the queue
     * logs that this function is called event gets updated at general surface level
     */
    public void setToNow() {
        done();
        for (int i = currentlyQueueing; i < queueElements.size(); i++) {
            queueElements.get(i).setDone(false);
            queueElements.get(i).setStarted(false);
        }
        done();
        currentEvent = queueElements.get(currentlyQueueing).startCondition;
        LOGGER.setLogLevel(RFLogger.Severity.INFO);
        LOGGER.log( "currentEvent :" + currentEvent);
    }

    /**
     * reset the queuer to factory settings
     * logs that this function is being called to general surface level
     */
    public void reset() {
        LOGGER.setLogLevel(RFLogger.Severity.INFO);
        LOGGER.log( "reset queuer");
        queueElements.clear();
        firstLoop = true;
        mustFinish = false;
        currentlyQueueing = 0;
        currentEvent = -1;
        mustStartCondition = -1;
        completeCurrentEvent = 0;
        delay = 0;
    }

    /**
     * is the entire queuer done
     * logs if queuer is done surface level
     */
    public boolean isFullfilled() {
        var newFulfilled = !queueElements.isEmpty() && currentEvent == queueElements.size() - 1;
        if(isFulfilled!=newFulfilled&&newFulfilled){
            LOGGER.setLogLevel(RFLogger.Severity.INFO);
            LOGGER.log( "queue finished!");
        }
        isFulfilled=newFulfilled;
        return isFulfilled;
    }

    /**
     * is it the first loop
     */
    public boolean isFirstLoop() {
        return firstLoop;
    }

    /**
     * recalculate the start condition after a previous event was changed to non optional/ optional
     *
     * @param p_ind          index of the event in question
     * @param p_asynchronous is the function asynchronous
     * @param p_Optional     is the function optional
     */
    private int recalcStartPosSkipOptional(int p_ind, boolean p_asynchronous, boolean p_Optional) {
        int startCondition = -1;
        boolean shouldFinish = false;
        if (p_ind < queueElements.size() && p_Optional && !queueElements.get(p_ind).getSkipOption()) {
            p_asynchronous = true;
        } else if (p_ind >= queueElements.size() && p_Optional) {
            p_asynchronous = true;
        }
        for (int i = 0; i < p_ind; i++) {
            if ((!queueElements.get(p_ind - i - 1).isAsynchronous() || queueElements.get(p_ind - i - 1).isMustFinish())
                    && (!queueElements.get(p_ind - i - 1).isOptional() || queueElements.get(p_ind - i - 1).getSkipOption())) {
                startCondition = p_ind - i - 1;
                if (p_asynchronous) {
                    if (i + 1 >= queueElements.size()) {
                        startCondition = -1;
                        break;
                    }
                    startCondition = queueElements.get(p_ind - i - 1).startCondition;
                }
                break;
            }
        }
        return startCondition;
    }

    /**
     * create new queueElement
     * logs which event was queued
     * @param p_asynchrnous is the function asynchronous
     * @param p_isOptional  is the function optional
     */
    private void createQueueElement(boolean p_asynchrnous, boolean p_isOptional) {
        int startCondition;
        if (!mustFinish) {
            startCondition = recalcStartPosSkipOptional(queueElements.size(), p_asynchrnous, p_isOptional);
            queueElements.add(new QueueElement(queueElements.size(), p_asynchrnous, startCondition, mustFinish, false, p_isOptional));
            LOGGER.setLogLevel(RFLogger.Severity.INFO);
            LOGGER.log( "event# : " + (queueElements.size() - 1) + ", StartCondition : " + startCondition);
        } else {
            mustFinish = false;
            startCondition = mustStartCondition;
            queueElements.add(new QueueElement(queueElements.size(), p_asynchrnous, startCondition, true));
            LOGGER.setLogLevel(RFLogger.Severity.INFO);
            LOGGER.log( "event# : " + (queueElements.size() - 1) + ", StartCondition : " + startCondition);
        }
    }

    /**
     * update which element is currently being queued(processed) and which element is currently being executed
     *log which element is currently being queued and which is being executed
     * @param p_done_condition is the function done
     * @param p_isOptional     is the function optional
     */
    private void updateQueuer(boolean p_done_condition, boolean p_isOptional) {
        //update which element is currently being queued
        if (currentlyQueueing >= queueElements.size() - 1) {
            currentlyQueueing = 0;
        } else {
            currentlyQueueing++;
        }
        //update which event is currently being executed
        if (!firstLoop) {
            if (queueElements.get(currentlyQueueing).isStarted() && !queueElements.get(currentlyQueueing).isDone()) {
                queueElements.get(currentlyQueueing).setDone(p_done_condition);
                if (p_done_condition) {
                    calculateCompleteCurrentEvent();
                    LOGGER.setLogLevel(RFLogger.Severity.INFO);
                    LOGGER.log( "currently Queueing event# : " + currentlyQueueing + "is Done, " + "completeEvents" + completeCurrentEvent);
                    if (currentlyQueueing > currentEvent && !queueElements.get(currentlyQueueing).isAsynchronous()) {
                        currentEvent = currentlyQueueing;
                        LOGGER.log( "currenty finished event# : " + currentEvent + "is Done, " + "completeEvents" + completeCurrentEvent);
                    }
                }
            }
        }
    }

    /**
     * calculate current event for must finish
     */
    private void calculateCompleteCurrentEvent() {
        for (int i = 0; i < queueElements.size(); i++) {
            if (queueElements.get(i).isDone()) {
                completeCurrentEvent = i;
                if (currentEvent < completeCurrentEvent) {
                    currentEvent = completeCurrentEvent;
                }
                //do nothing
            } else {
                break;
            }
        }
    }

    /**
     * set the delay of the next queued element
     */
    public void addDelay(double p_delay) {
        delay = p_delay;
    }

    /**
     * Wait for all currently active functions to finish
     */
    public void waitForFinish() {
        waitForFinish(queueElements.size() - 1);
    }

    public void waitForFinish(int p_startCondition) {
        if (firstLoop) {
            mustFinish = true;
            mustStartCondition = p_startCondition;
        }
    }
}