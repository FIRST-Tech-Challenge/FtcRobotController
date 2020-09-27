package org.firstinspires.ftc.teamcode.action;

import org.firstinspires.ftc.teamcode.playmaker.Action;
import org.firstinspires.ftc.teamcode.playmaker.RobotHardware;

/**
 * Created by djfig on 12/12/2016.
 */
public class WaitAction implements Action {
    double time;
    double startTime;
    double endTime;

    /**
     *
     * @param time Time in milliseconds for the action to wait.
     */
    public WaitAction(double time){
        this.time = time;
    }

    @Override
    public void init(RobotHardware hardware) {
        startTime = System.currentTimeMillis();
        endTime = System.currentTimeMillis() + time;
    }

    @Override
    public boolean doAction(RobotHardware hardware){
        return (System.currentTimeMillis() >= endTime);
    }

    @Override
    public Double progress() {
        return (System.currentTimeMillis()-startTime)/time;
    }

    @Override
    public String progressString() {
        String elapsedTime = String.format("%.3f", (System.currentTimeMillis() - startTime)/1000.0);
        String totalTime = String.format("%.3f", time/1000.0);
        return String.format("%s/%ss", elapsedTime, totalTime);
    }
}
