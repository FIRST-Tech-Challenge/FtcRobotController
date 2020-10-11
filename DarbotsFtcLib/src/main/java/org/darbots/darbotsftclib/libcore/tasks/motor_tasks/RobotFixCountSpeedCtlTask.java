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

package org.darbots.darbotsftclib.libcore.tasks.motor_tasks;

import org.darbots.darbotsftclib.libcore.templates.motor_related.RobotMotorTaskCallBack;

public class RobotFixCountSpeedCtlTask extends RobotFixedSpeedTask {
    protected int m_Count;
    protected boolean m_CountCtl = false;

    public RobotFixCountSpeedCtlTask(int Count, double Speed, RobotMotorTaskCallBack TaskCallBack, boolean CountCtl) {
        super(0,Speed,TaskCallBack);
        this.m_Count = Count;
        this.setSpeed(Speed);
        this.m_CountCtl = CountCtl;
    }
    public RobotFixCountSpeedCtlTask(RobotFixCountSpeedCtlTask FixCountSpeedCtlTask){
        super(FixCountSpeedCtlTask);
        this.m_Count = FixCountSpeedCtlTask.m_Count;
        this.m_CountCtl = FixCountSpeedCtlTask.m_CountCtl;
    }

    public boolean isCountCtl(){
        return this.m_CountCtl;
    }

    public void setCountCtl(boolean CountCtl){
        this.m_CountCtl = CountCtl;
    }

    public int getCounts(){
        return this.m_Count;
    }

    public void setCounts(int Count){
        this.m_Count = Count;
        if(this.isBusy()){
            this.fixCounts();
        }
    }

    protected void fixCounts(){
        double rev = ((double) super.getStartCount() + this.getCounts() - super.getMotorController().getMotor().getCurrentCount()) / super.getMotorController().getMotor().getMotorType().getCountsPerRev();
        double speed = Math.abs(this.getSpeed() * super.getMotorController().getMotor().getMotorType().getRevPerSec());
        double FineTime = Math.abs(rev / speed);
        if(super.isTimeControlEnabled()) {
            FineTime *= Math.abs(super.getTimeOutFactor());
        }else{
            FineTime = 0;
        }
        double FineTimeAfter = (FineTime == 0 ? 0 : super.getSecondsSinceStart() + FineTime);
        this.setTimeInSeconds(FineTimeAfter);
    }

    @Override
    public void setSpeed(double Speed){
        super.setSpeed(fixSpeed(Speed));
        if(this.isBusy()) {
            this.fixCounts();
        }
    }

    protected double fixSpeed(double speed){
        if(this.isBusy()) {
            if (this.m_Count + super.getStartCount() - super.getMotorController().getMotor().getCurrentCount() < 0) {
                return -Math.abs(speed);
            } else {
                return Math.abs(speed);
            }
        }else{
            if(this.m_Count < 0){
                return -Math.abs(speed);
            }else{
                return Math.abs(speed);
            }
        }
    }

    @Override
    protected void __startTask() {
        this.fixCounts();
        this.setSpeed(fixSpeed(this.getSpeed()));
        super.__startTask();
        if(this.m_Count == 0){
            this.endTask(true);
        }
    }

    @Override
    public void updateStatus(){
        if(this.m_CountCtl) {
            if (m_Count > 0) {
                if (this.getMotorController().getMotor().getCurrentCount() >= super.getStartCount() + this.m_Count) {
                    this.endTask(false);
                }
            } else if(m_Count < 0) {
                if (this.getMotorController().getMotor().getCurrentCount() <= super.getStartCount() + this.m_Count) {
                    this.endTask(false);
                }
            }else{ //m_Count == 0
                this.endTask(false);
            }
        }
        super.updateStatus();
    }

    @Override
    public double getProgressRatio(){
        if(!this.isCountCtl()) {
            double countMoved = this.getMotorController().getMotor().getCurrentCount() - super.getStartCount();
            return Math.abs(countMoved / this.m_Count);
        }else{
            return super.getProgressRatio();
        }
    }

    @Override
    public String getTaskDetailString() {
        String result="TaskType: RobotFixCountSpeedCtlTask, ";
        result += "TimeInSeconds: " + this.getTimeInSeconds() + ", ";
        result += "Speed: " + this.getSpeed() + ", ";
        result += "Count: " + this.getCounts() + ", ";
        result += "CountControl: " + (this.isCountCtl() ? "Enabled" : "Disabled");
        return result;
    }
}
