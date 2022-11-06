package org.firstinspires.ftc.teamcode.Components;

import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.logger;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.op;

public class QueueElement {
    //place in line
    private int queuePos = 0;
    //after which item is done u can run
    public int startCondition = 0;
    //if event is asynchronous(if should be sequential or ignore)
    private boolean asynchronous=false;
    //if started
    private boolean started = false;
    //if done
    private boolean isDone = false;
    //For delay
    private double readyTime = 1000;
    public QueueElement(int queueNum, boolean p_asynchronous, int p_startCondition) {
        queuePos = queueNum;
        asynchronous = p_asynchronous;
        startCondition = p_startCondition;
    }

    public boolean isReady(int currentEvent, boolean extraCondition){
        //is it this elements turn to run
        if(currentEvent>=startCondition&&!isDone){
            // update when start for delay logic
            if(readyTime>op.getRuntime()){
                readyTime = op.getRuntime();
            }
            if(extraCondition) {
                return true;
            }
            else{
                logger.log("/RobotLogs/GeneralRobot","readyTime =" + readyTime);
                return false;
            }
        }
        else{
            return false;
        }
    }
    public boolean isDone(){return isDone;}
    public void setDone(boolean p_isDone){
        isDone=p_isDone;
    }
    public void setStarted(boolean p_started){
        started = p_started;
    }
    public boolean isStarted(){
        return started;
    }
    public boolean isAsynchronous(){return asynchronous;}
    public double getReadyTime(){return readyTime;}
}
