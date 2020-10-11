package org.darbots.darbotsftclib.libcore.tasks.servo_tasks.motor_powered_servo_tasks;

import androidx.annotation.NonNull;

import org.darbots.darbotsftclib.libcore.sensors.servos.motor_powered_servos.RobotServoUsingMotor_WithLimitSwitch;
import org.darbots.darbotsftclib.libcore.tasks.motor_tasks.RobotFixCountSpeedCtlTask;
import org.darbots.darbotsftclib.libcore.templates.servo_related.motor_powered_servos.RobotServoUsingMotorCallBack;
import org.darbots.darbotsftclib.libcore.templates.servo_related.motor_powered_servos.RobotServoUsingMotorTask;

public class TargetPosSpeedCtlTask extends RobotServoUsingMotorTask {
    private double m_TargetPos;
    private double m_Power;

    public TargetPosSpeedCtlTask(RobotServoUsingMotorCallBack taskCallBack, double targetPos, double power) {
        super(taskCallBack);
        this.m_TargetPos = targetPos;
        this.m_Power = power;
    }

    public TargetPosSpeedCtlTask(@NonNull TargetPosSpeedCtlTask oldTask) {
        super(oldTask);
        this.m_TargetPos = oldTask.m_TargetPos;
        this.m_Power = oldTask.m_Power;
    }

    public double getPower(){
        return this.m_Power;
    }

    public void setPower(double power){
        this.m_Power = power;
        if(this.isBusy()){
            if(this.getServoUsingMotor().getMotorController().getCurrentTask() != null){
                RobotFixCountSpeedCtlTask fixCountSpeedCtlTask = (RobotFixCountSpeedCtlTask) this.getServoUsingMotor().getMotorController().getCurrentTask();
                fixCountSpeedCtlTask.setSpeed(power);
            }
        }
    }

    public double getTargetPos(){
        return this.m_TargetPos;
    }

    public void setTargetPos(double targetPos){
        this.m_TargetPos = targetPos;
        if(this.isBusy()){
            if(this.getServoUsingMotor().getMotorController().getCurrentTask() != null){
                RobotFixCountSpeedCtlTask fixCountSpeedCtlTask = (RobotFixCountSpeedCtlTask) this.getServoUsingMotor().getMotorController().getCurrentTask();
                double deltaPos = this.getTargetPos() - super.getTaskStartPos();
                int deltaCount = (int) Math.round(deltaPos * this.getServoUsingMotor().getMotorController().getMotor().getMotorType().getCountsPerRev());
                fixCountSpeedCtlTask.setCounts(deltaCount);
            }
        }
    }

    @Override
    protected void __startTask() {
        if((getTargetPos() > this.getServoUsingMotor().getMaxPos() || getTargetPos() < this.getServoUsingMotor().getMinPos()) && this.getServoUsingMotor().isBorderControl()){
            this.endTask(false);
        }
        double deltaPos = this.getTargetPos() - super.getTaskStartPos();
        int deltaCount = (int) Math.round(deltaPos * this.getServoUsingMotor().getMotorController().getMotor().getMotorType().getCountsPerRev());
        RobotFixCountSpeedCtlTask fixCountSpeedCtlTask = new RobotFixCountSpeedCtlTask(deltaCount,this.getPower(),null,true);
        this.getServoUsingMotor().getMotorController().replaceTask(fixCountSpeedCtlTask);
    }

    @Override
    protected void __finishTask() {

    }

    @Override
    public void updateStatus(){
        super.updateStatus();
        if(this.isBusy() && this.getServoUsingMotor() instanceof RobotServoUsingMotor_WithLimitSwitch) {
            RobotServoUsingMotor_WithLimitSwitch mServo = (RobotServoUsingMotor_WithLimitSwitch) this.getServoUsingMotor();
            if (mServo.getMinSwitch() != null) {
                if (mServo.getMinSwitch().isPressed() && this.m_TargetPos <= this.getTaskStartPos()) {
                    this.getServoUsingMotor().adjustCurrentPosition(this.getServoUsingMotor().getMinPos());
                    this.endTask(false);
                }
            }
            if (mServo.getMaxSwitch() != null) {
                if (mServo.getMaxSwitch().isPressed() && this.m_TargetPos >= this.getTaskStartPos()) {
                    this.getServoUsingMotor().adjustCurrentPosition(this.getServoUsingMotor().getMaxPos());
                    this.endTask(false);
                }
            }
        }
    }

    @Override
    protected void __recalculateMotorCounts() {
        double deltaPos = this.getTargetPos() - super.getServoUsingMotor().getCurrentPosition();
        int deltaCount = (int) Math.round(deltaPos * this.getServoUsingMotor().getMotorController().getMotor().getMotorType().getCountsPerRev());
        RobotFixCountSpeedCtlTask fixCountSpeedCtlTask = new RobotFixCountSpeedCtlTask(deltaCount,this.getPower(),null,true);
        this.getServoUsingMotor().getMotorController().replaceTask(fixCountSpeedCtlTask);
    }

    @Override
    public String getTaskDetailString() {
        String result = "TaskType: TargetPosSpeedCtlTask, ";
        result += "TargetPos: " + this.getTargetPos() + ", ";
        result += "Power: " + this.getPower();
        return result;
    }
}
