package org.darbots.darbotsftclib.libcore.integratedfunctions;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.darbots.darbotsftclib.libcore.runtime.GlobalRegister;
import org.darbots.darbotsftclib.libcore.templates.RobotNonBlockingDevice;

public abstract class LoopableTimer implements RobotNonBlockingDevice {
    private ElapsedTime m_Time;
    private double m_Duration = 0;
    private boolean m_TimerEnable = false;

    protected abstract void run();

    public LoopableTimer(double duration_sec){
        this.m_Duration = duration_sec;
        this.m_Time = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
    }

    public LoopableTimer(LoopableTimer anotherTimer){
        this.m_Duration = anotherTimer.m_Duration;
        this.m_Time = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
    }

    public boolean isScheduled(){
        return this.m_TimerEnable;
    }

    public double getDuration(){
        return this.m_Duration;
    }

    public void setDuration(double durationInSec){
        this.m_Duration = durationInSec;
    }

    public double getDurationPassed(){
        return this.m_Time.seconds();
    }

    public double getSecondsLeft(){
        return this.getDuration() - this.getDurationPassed();
    }

    public void start(){
        if(this.m_TimerEnable){
            return;
        }
        this.m_TimerEnable = true;
        this.m_Time.reset();
    }

    public void stop(){
        this.m_TimerEnable = false;
    }

    @Override
    public boolean isBusy() {
        return this.isScheduled();
    }

    @Override
    public void updateStatus() {
        if(this.isScheduled()){
            if(this.getSecondsLeft()<=0){
                this.run();
                this.stop();
            }
        }
    }

    @Override
    public void waitUntilFinish() {
        while(this.isBusy()){
            if(GlobalRegister.runningOpMode != null){
                if(GlobalRegister.runningOpMode.isStarted() && (!GlobalRegister.runningOpMode.opModeIsActive())){
                    return;
                }
            }
            this.updateStatus();
        }
    }
}
