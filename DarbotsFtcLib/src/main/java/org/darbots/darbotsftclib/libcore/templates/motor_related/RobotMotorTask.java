/*
MIT License

Copyright (c) 2018 DarBots Collaborators

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

*/

package org.darbots.darbotsftclib.libcore.templates.motor_related;


import androidx.annotation.NonNull;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.darbots.darbotsftclib.libcore.runtime.GlobalRegister;
import org.darbots.darbotsftclib.libcore.runtime.GlobalUtil;
import org.darbots.darbotsftclib.libcore.sensors.motors.RobotMotorController;
import org.darbots.darbotsftclib.libcore.templates.RobotNonBlockingDevice;
import org.darbots.darbotsftclib.libcore.templates.log.LogLevel;

public abstract class RobotMotorTask implements RobotNonBlockingDevice {
    private RobotMotorController m_Controller;
    private RobotMotorTaskCallBack m_TaskCallBack = null;
    private int m_StartCount = 0;
    private ElapsedTime m_Time;
    private boolean m_IsWorking = false;
    private double m_TimeOutFactor = 0;
    private boolean m_TimeoutControl = false;

    public RobotMotorTask(RobotMotorTaskCallBack TaskCallBack){
        this.m_TaskCallBack = TaskCallBack;
        this.m_Time = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
        m_IsWorking = false;
    }
    public RobotMotorTask(@NonNull RobotMotorTask MotorTask){
        this.m_Controller = MotorTask.m_Controller;
        this.m_TaskCallBack = MotorTask.m_TaskCallBack;
        this.m_StartCount = 0;
        this.m_Time = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
        this.m_IsWorking = false;
        this.m_TimeOutFactor = MotorTask.m_TimeOutFactor;
        this.m_TimeoutControl = MotorTask.m_TimeoutControl;
    }
    public boolean isTimeControlEnabled(){
        return this.m_TimeoutControl;
    }
    public void setTimeControlEnabled(boolean timeOutControlEnabled){
        this.m_TimeoutControl = timeOutControlEnabled;
    }
    public double getTimeOutFactor(){
        return this.m_TimeOutFactor;
    }
    public void setTimeOutFactor(double timeOutFactor){
        this.m_TimeOutFactor = timeOutFactor;
    }
    public RobotMotorController getMotorController(){
        return this.m_Controller;
    }
    public void setMotorController(@NonNull RobotMotorController Controller){
        this.m_Controller = Controller;
    }

    public RobotMotorTaskCallBack getTaskCallBack(){
        return this.m_TaskCallBack;
    }

    public void setTaskCallBack(RobotMotorTaskCallBack TaskCallBack){
        this.m_TaskCallBack = TaskCallBack;
    }


    public void startTask(){
        if(this.m_IsWorking) {
            return;
        }
        GlobalUtil.addLog("RobotMotorTask","BeforeTaskStatus", this.getMotorController().getMotor().getMotorStatusString(), LogLevel.DEBUG);
        GlobalUtil.addLog("RobotMotorTask","TaskInfo", this.getTaskDetailString(), LogLevel.DEBUG);
        this.m_IsWorking = true;
        this.m_StartCount = this.getMotorController().getMotor().getCurrentCount();
        this.m_Time.reset();
        this.__startTask();
        GlobalUtil.addLog("RobotMotorTask","DuringTaskStatus", this.getMotorController().getMotor().getMotorStatusString(), LogLevel.DEBUG);
    }
    protected abstract void __startTask();
    public void endTask(boolean timedOut){
        if(!this.m_IsWorking){
            return;
        }
        GlobalUtil.addLog("RobotMotorTask","AfterTask","Task ends, " + (timedOut ? "timed out!" : "normally finished"), LogLevel.DEBUG);
        this.m_IsWorking = false;
        int endCount = this.getMotorController().getMotor().getCurrentCount();
        int deltaCount = endCount - this.m_StartCount;
        double timeUsed = this.m_Time.seconds();
        if(this.m_TaskCallBack != null) {
            this.m_TaskCallBack.finishRunning(this.m_Controller,timedOut,timeUsed,deltaCount);
        }
        GlobalUtil.addLog("RobotMotorTask","AfterTaskStatus", this.getMotorController().getMotor().getMotorStatusString(), LogLevel.DEBUG);
        this.m_Controller.__checkTasks();
    }

    public int getStartCount(){
        return this.m_StartCount;
    }

    public abstract double getProgressRatio();

    @Override
    public boolean isBusy(){
        return this.m_IsWorking;
    }

    @Override
    public void waitUntilFinish(){
        while(this.isBusy()){
            if(GlobalRegister.runningOpMode != null){
                if(GlobalRegister.runningOpMode.isStarted() && (!GlobalRegister.runningOpMode.opModeIsActive())){
                    return;
                }
            }
            this.updateStatus();
        }
    }

    public double getSecondsSinceStart(){
        if(this.isBusy()) {
            return this.m_Time.seconds();
        }else{
            return 0;
        }
    }

    public abstract String getTaskDetailString();

}
