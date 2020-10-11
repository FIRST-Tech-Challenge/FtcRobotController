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

package org.darbots.darbotsftclib.libcore.sensors.motors;

import androidx.annotation.NonNull;

import org.darbots.darbotsftclib.libcore.runtime.GlobalRegister;
import org.darbots.darbotsftclib.libcore.templates.motor_related.RobotMotor;
import org.darbots.darbotsftclib.libcore.templates.RobotNonBlockingDevice;
import org.darbots.darbotsftclib.libcore.templates.motor_related.RobotMotorTask;

import java.util.ArrayList;


public class RobotMotorController implements RobotNonBlockingDevice {
    private RobotMotor m_Motor;
    protected ArrayList<RobotMotorTask> m_TaskLists;
    private double m_TimeOutFactor = 0;
    private boolean m_TimeoutControl = false;

    public RobotMotorController(@NonNull RobotMotor Motor, boolean timeOutControlEnabled, double timeOutFactor){
        this.m_Motor = Motor;
        m_TaskLists = new ArrayList<RobotMotorTask>();
        this.m_TimeoutControl = timeOutControlEnabled;
        this.m_TimeOutFactor = timeOutFactor;
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
    public RobotMotor getMotor(){
        return this.m_Motor;
    }
    public void setMotor(@NonNull RobotMotor Motor){
        this.m_Motor = Motor;
    }

    @Override
    public boolean isBusy() {
        return (!this.m_TaskLists.isEmpty());
    }

    @Override
    public void updateStatus() {
        if(!this.m_TaskLists.isEmpty()){
            if(this.m_TaskLists.get(0).isBusy())
                this.m_TaskLists.get(0).updateStatus();
        }
        this.getMotor().updateStatus();
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

    public void __checkTasks(){
        if(this.m_TaskLists.isEmpty()){
            this.stopMotor();
            return;
        }
        else if(!this.m_TaskLists.get(0).isBusy()){
            this.deleteCurrentTask();
        }
    }

    public void addTask(@NonNull RobotMotorTask MotorTask){
        this.m_TaskLists.add(MotorTask);
        this.scheduleTasks();
    }

    public void replaceTask(@NonNull RobotMotorTask MotorTask){
        if(!this.m_TaskLists.isEmpty() && this.m_TaskLists.get(0).isBusy()){
            this.m_TaskLists.get(0).endTask(true);
        }
        this.m_TaskLists.clear();
        this.m_TaskLists.add(MotorTask);
        this.scheduleTasks();
    }

    public void deleteCurrentTask(){
        if(!this.m_TaskLists.isEmpty()) {
            if (this.m_TaskLists.get(0).isBusy()) {
                this.m_TaskLists.get(0).endTask(true);
            }
            this.m_TaskLists.remove(0);
            scheduleTasks();
        }else{
            this.stopMotor();
        }
    }

    public ArrayList<RobotMotorTask> getTaskLists(){
        return this.m_TaskLists;
    }

    protected void scheduleTasks(){
        if(!this.m_TaskLists.isEmpty()) {
            if(!this.m_TaskLists.get(0).isBusy()) {
                this.m_TaskLists.get(0).setMotorController(this);
                this.m_TaskLists.get(0).setTimeControlEnabled(this.isTimeControlEnabled());
                this.m_TaskLists.get(0).setTimeOutFactor(this.getTimeOutFactor());
                this.m_TaskLists.get(0).startTask();
            }
        }else{ //if(this.m_TaskLists.isEmpty()){
            stopMotor();
        }
    }

    public void deleteAllTasks(){
        if(!this.m_TaskLists.isEmpty()){
            if(this.m_TaskLists.get(0).isBusy()){
                this.m_TaskLists.get(0).endTask(true);
            }
            this.m_TaskLists.clear();
        }
        this.stopMotor();
    }

    public RobotMotorTask getCurrentTask(){
        return this.m_TaskLists.isEmpty() ? null : this.m_TaskLists.get(0);
    }

    protected void stopMotor(){
        this.m_Motor.setCurrentMovingType(RobotMotor.MovingType.withSpeed);
        this.m_Motor.setPower(0);
    }
}
