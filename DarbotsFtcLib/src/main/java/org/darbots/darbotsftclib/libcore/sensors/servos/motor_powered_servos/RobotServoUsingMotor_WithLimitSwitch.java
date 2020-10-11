package org.darbots.darbotsftclib.libcore.sensors.servos.motor_powered_servos;

import com.qualcomm.robotcore.hardware.TouchSensor;

import org.darbots.darbotsftclib.libcore.sensors.motors.RobotMotorController;

public class RobotServoUsingMotor_WithLimitSwitch extends RobotServoUsingMotor {
    private TouchSensor m_MinSwitch, m_MaxSwitch;

    public RobotServoUsingMotor_WithLimitSwitch(RobotMotorController motorController, double currentPosition, double minPosition, double maxPosition, TouchSensor minSwitch, TouchSensor maxSwitch) {
        super(motorController, currentPosition, minPosition, maxPosition);
        this.m_MinSwitch = minSwitch;
        this.m_MaxSwitch = maxSwitch;
    }
    public TouchSensor getMinSwitch(){
        return this.m_MinSwitch;
    }
    public void setMinSwitch(TouchSensor minSwitch){
        this.m_MinSwitch = minSwitch;
    }
    public TouchSensor getMaxSwitch(){
        return this.m_MaxSwitch;
    }
    public void setMaxSwitch(TouchSensor maxSwitch){
        this.m_MaxSwitch = maxSwitch;
    }
    @Override
    public void updateStatus(){
        super.updateStatus();
        if(this.isBusy()) {
            if (this.m_MinSwitch != null) {
                if (this.m_MinSwitch.isPressed()) {
                    this.adjustCurrentPosition(this.getMinPos());
                }
            }
            if (this.m_MaxSwitch != null) {
                if (this.m_MaxSwitch.isPressed()) {
                    this.adjustCurrentPosition(this.getMaxPos());
                }
            }
        }
    }
}
