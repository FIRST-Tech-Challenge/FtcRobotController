package org.darbots.darbotsftclib.libcore.templates.servo_related.motor_powered_servos;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.darbots.darbotsftclib.libcore.runtime.GlobalRegister;
import org.darbots.darbotsftclib.libcore.runtime.GlobalUtil;
import org.darbots.darbotsftclib.libcore.sensors.servos.motor_powered_servos.RobotServoUsingMotor;
import org.darbots.darbotsftclib.libcore.templates.RobotNonBlockingDevice;
import org.darbots.darbotsftclib.libcore.templates.log.LogLevel;

public abstract class RobotServoUsingMotorTask implements RobotNonBlockingDevice {
    private boolean m_IsBusy = false;
    private double m_StartPos;
    private ElapsedTime m_Time = null;
    private RobotServoUsingMotor m_Servo = null;
    private RobotServoUsingMotorCallBack m_CallBack = null;
    private RobotServoUsingMotorPreCheck m_Precheck = null;

    public RobotServoUsingMotorTask(RobotServoUsingMotorCallBack taskCallBack){
        this.m_IsBusy = false;
        this.m_Time = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
        this.m_CallBack = taskCallBack;
    }
    public RobotServoUsingMotorTask(@NonNull RobotServoUsingMotorTask oldTask){
        this.m_IsBusy = false;
        this.m_Servo = oldTask.m_Servo;
        this.m_Time = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
        this.m_CallBack = oldTask.m_CallBack;
    }
    public RobotServoUsingMotorCallBack getTaskCallBack(){
        return this.m_CallBack;
    }
    public void setTaskCallBack(RobotServoUsingMotorCallBack taskCallBack){
        this.m_CallBack = taskCallBack;
    }
    public RobotServoUsingMotorPreCheck getTaskPreCheck(){
        return this.m_Precheck;
    }
    public void setTaskPreCheck(RobotServoUsingMotorPreCheck taskPreCheck){
        this.m_Precheck = taskPreCheck;
    }
    public RobotServoUsingMotor getServoUsingMotor(){
        return this.m_Servo;
    }
    public void setServoUsingMotor(@NonNull RobotServoUsingMotor servoUsingMotor){
        this.m_Servo = servoUsingMotor;
    }
    protected abstract void __startTask();
    protected abstract void __finishTask();
    public void startTask(){
        //GlobalRegister.runningOpMode.getRobotCore().getLogger().addLog("RobotServoUsingMotorTask","BeforeTask",);
        if(this.isBusy()){
            return;
        }
        GlobalUtil.addLog("RobotServoUsingMotorTask","BeforeTaskStatus",this.getServoUsingMotor().getStatusString(), LogLevel.DEBUG);
        GlobalUtil.addLog("RobotServoUsingMotorTask","TaskInfo", this.getTaskDetailString(), LogLevel.DEBUG);
        this.m_IsBusy = true;
        this.m_StartPos = this.getServoUsingMotor().getCurrentPosition();
        this.m_Time.reset();
        this.__startTask();
    }
    public void endTask(boolean timeOut){
        if(!this.isBusy()){
            return;
        }
        GlobalUtil.addLog("RobotServoUsingMotorTask","AfterTask","Task ends, " + (timeOut ? "timed out!" : "normally finished"), LogLevel.DEBUG);
        double timeConsumed = this.m_Time.seconds();
        this.m_IsBusy = false;
        this.__finishTask();
        GlobalUtil.addLog("RobotServoUsingMotorTask","AfterTaskStatus",this.getServoUsingMotor().getStatusString(), LogLevel.DEBUG);
        if(this.m_CallBack != null){
            this.m_CallBack.JobFinished(timeOut,this,this.m_StartPos,timeConsumed);
        }
        this.getServoUsingMotor().__checkTasks();
    }
    public double getTaskStartPos(){
        return this.m_StartPos;
    }
    @Override
    public void updateStatus(){
        if(this.getServoUsingMotor() != null){
            this.getServoUsingMotor().getMotorController().updateStatus();
            if(!this.getServoUsingMotor().getMotorController().isBusy()){
                this.endTask(false);
            }
        }
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
    public void servoPositionAdjusted(){
        if(this.isBusy()){
            this.__recalculateMotorCounts();
        }
    }
    protected abstract void __recalculateMotorCounts();
    @Override
    public boolean isBusy(){
        return this.m_IsBusy;
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
