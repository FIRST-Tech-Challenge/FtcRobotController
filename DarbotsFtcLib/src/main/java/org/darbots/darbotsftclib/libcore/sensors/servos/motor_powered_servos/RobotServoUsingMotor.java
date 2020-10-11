package org.darbots.darbotsftclib.libcore.sensors.servos.motor_powered_servos;

import androidx.annotation.NonNull;

import org.darbots.darbotsftclib.libcore.runtime.GlobalRegister;
import org.darbots.darbotsftclib.libcore.sensors.motors.RobotMotorController;
import org.darbots.darbotsftclib.libcore.templates.RobotNonBlockingDevice;
import org.darbots.darbotsftclib.libcore.templates.motor_related.MotorType;
import org.darbots.darbotsftclib.libcore.templates.servo_related.motor_powered_servos.RobotServoUsingMotorPreCheck;
import org.darbots.darbotsftclib.libcore.templates.servo_related.motor_powered_servos.RobotServoUsingMotorTask;

import java.util.ArrayList;

public class RobotServoUsingMotor implements RobotNonBlockingDevice {
    private ArrayList<RobotServoUsingMotorTask> m_TaskList;
    private int m_CountAtZeroPos;
    private double m_MinPos;
    private double m_MaxPos;
    private RobotMotorController m_MotorController;
    private RobotServoUsingMotorPreCheck m_DefaultPreCheck = null;
    private boolean m_BorderControl;

    public RobotServoUsingMotor(RobotMotorController motorController, double currentPosition, double minPosition, double maxPosition){
        this.m_MaxPos = maxPosition;
        this.m_MinPos = minPosition;
        this.m_MotorController = motorController;
        this.m_BorderControl = true;
        this.m_TaskList = new ArrayList<>();
        this.__adjustInitialPos(motorController.getMotor().getCurrentCount(),currentPosition,motorController.getMotor().getMotorType());
    }

    public RobotServoUsingMotorPreCheck getDefaultPreCheck(){
        return this.m_DefaultPreCheck;
    }

    public void setDefaultPreCheck(RobotServoUsingMotorPreCheck preCheck){
        this.m_DefaultPreCheck = preCheck;
    }

    public boolean isBorderControl(){
        return this.m_BorderControl;
    }

    public void setBorderControl(boolean enabled){
        this.m_BorderControl = enabled;
    }

    public double getCurrentPosition(){
        return (this.getMotorController().getMotor().getCurrentCount() - this.m_CountAtZeroPos) / this.getMotorController().getMotor().getMotorType().getCountsPerRev();
    }
    public double getCurrentPercent(){
        return this.percentFromPos(getCurrentPosition());
    }

    public double getMinPos(){
        return this.m_MinPos;
    }
    public void setMinPos(double minPos){
        this.m_MinPos = minPos;
    }
    public double getMaxPos(){
        return this.m_MaxPos;
    }
    public void setMaxPos(double maxPos){
        this.m_MaxPos = maxPos;
    }
    public RobotMotorController getMotorController(){
        return this.m_MotorController;
    }
    public void __checkTasks(){
        if(this.m_TaskList.isEmpty()){
            return;
        }
        else if(!this.m_TaskList.get(0).isBusy()){
            this.deleteCurrentTask();
        }
    }

    public void addTask(@NonNull RobotServoUsingMotorTask ServoTask){
        this.m_TaskList.add(ServoTask);
        this.scheduleTasks();
    }

    public void replaceTask(@NonNull RobotServoUsingMotorTask ServoTask){
        if(!this.m_TaskList.isEmpty() && this.m_TaskList.get(0).isBusy()){
            this.m_TaskList.get(0).endTask(true);
        }
        this.m_TaskList.clear();
        this.m_TaskList.add(ServoTask);
        this.scheduleTasks();
    }

    public void deleteCurrentTask(){
        if(!this.m_TaskList.isEmpty()) {
            if (this.m_TaskList.get(0).isBusy()) {
                this.m_TaskList.get(0).endTask(true);
            }
            this.m_TaskList.remove(0);
            scheduleTasks();
        }else{
            return;
        }
    }

    public void deleteAllTasks(){
        if(!this.m_TaskList.isEmpty()){
            if(this.m_TaskList.get(0).isBusy()){
                this.m_TaskList.get(0).endTask(true);
            }
            this.m_TaskList.clear();
        }
    }

    public RobotServoUsingMotorTask getCurrentTask(){
        return this.m_TaskList.isEmpty() ? null : this.m_TaskList.get(0);
    }
    public ArrayList<RobotServoUsingMotorTask> getTaskLists(){
        return this.m_TaskList;
    }

    protected void scheduleTasks(){
        if(!this.m_TaskList.isEmpty()) {
            if(!this.m_TaskList.get(0).isBusy()) {
                this.m_TaskList.get(0).setServoUsingMotor(this);
                if(this.m_TaskList.get(0).getTaskPreCheck() == null && this.getDefaultPreCheck() != null){
                    this.m_TaskList.get(0).setTaskPreCheck(this.getDefaultPreCheck());
                }
                this.m_TaskList.get(0).startTask();
            }
        }else{ //if(this.m_TaskLists.isEmpty()){
            this.m_MotorController.deleteAllTasks();
            return;
        }
    }
    @Override
    public boolean isBusy() {
        return (!this.m_TaskList.isEmpty());
    }

    @Override
    public void updateStatus() {
        if(!this.m_TaskList.isEmpty()){
            if(this.m_TaskList.get(0).isBusy()){
                this.m_TaskList.get(0).updateStatus();
            }
            if(this.m_MotorController != null) {
                this.m_MotorController.updateStatus();
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
    protected void __adjustInitialPos(int Count, double designatedPos, MotorType motorType){
        int correspondingDesignatedCount = (int) Math.round(designatedPos * motorType.getCountsPerRev());
        this.m_CountAtZeroPos = Count - correspondingDesignatedCount;
    }

    public void adjustCurrentPosition(double currentPosition){
        this.__adjustInitialPos(this.m_MotorController.getMotor().getCurrentCount(),currentPosition,this.m_MotorController.getMotor().getMotorType());
        if(!this.m_TaskList.isEmpty()){
            this.m_TaskList.get(0).servoPositionAdjusted();
        }
    }

    public double percentFromPos(double pos){
        return (pos - this.getMinPos()) / (this.getMaxPos() - this.getMinPos()) * 100;
    }
    public double posFromPercent(double percent){
        return (percent / 100 * (this.getMaxPos() - this.getMinPos())) + this.getMinPos();
    }
    public String getStatusString(){
        String result = "CurrentPos: " + this.getCurrentPosition() + ", ";
        result += "CurrentPct: " + this.getCurrentPercent() + ", ";
        result += "MinPos: " + this.getMinPos() + ", ";
        result += "MaxPos: " + this.getMaxPos() + ", ";
        result += "BorderControl: " + (this.isBorderControl() ? "Enabled" : "Disabled");
        return result;
    }
}
