package org.darbots.darbotsftclib.libcore.sensors.servos;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.darbots.darbotsftclib.libcore.runtime.GlobalRegister;
import org.darbots.darbotsftclib.libcore.templates.RobotNonBlockingDevice;
import org.darbots.darbotsftclib.libcore.templates.servo_related.ServoType;

public class TimeControlledServo implements RobotNonBlockingDevice {
    private Servo m_Servo;
    private ServoType m_ServoType;
    private double m_LastPosition;
    private double m_TargetPosition;
    private boolean m_ToPositionEnabled;
    private double m_EstimatedTaskTime;
    private double m_Speed;
    private ElapsedTime m_TaskTime = null;

    public TimeControlledServo(Servo servo, ServoType servoType, double initialPosition, boolean setServoPositionAtStart){
        this.m_Servo = servo;
        this.m_ServoType = servoType;
        this.m_LastPosition = initialPosition;
        this.m_TargetPosition = 0;
        this.m_ToPositionEnabled = false;
        this.m_EstimatedTaskTime = 0;
        this.m_TaskTime = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
        this.m_Speed = 0;
        if(setServoPositionAtStart){
            this.m_Servo.setPosition(initialPosition);
        }
    }

    public TimeControlledServo(TimeControlledServo tmServo){
        this.m_Servo = tmServo.m_Servo;
        this.m_ServoType = tmServo.m_ServoType;
        this.m_LastPosition = tmServo.m_LastPosition;
        this.m_TargetPosition = 0;
        this.m_ToPositionEnabled = false;
        this.m_EstimatedTaskTime = 0;
        this.m_Speed = 0;
        this.m_TaskTime = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
    }

    public void adjustLastPosition(double lastPosition){
        this.m_LastPosition = lastPosition;
    }

    public Servo getServo(){
        return this.m_Servo;
    }
    public void setServo(Servo servo){
        this.m_Servo = servo;
    }
    public ServoType getServoType(){
        return this.m_ServoType;
    }
    public void setServoType(ServoType servoType){
        this.m_ServoType = servoType;
    }
    public double getLastPosition(){
        return this.m_LastPosition;
    }

    public double getLastAngle(){
        return this.getServoType().getDegFromPosition(this.m_LastPosition);
    }

    public double getCurrentPosition(){
        if(!this.isBusy()){
            return this.getLastPosition();
        }else{
            return (this.getTaskProgressRatio() * (this.getTargetPosition() - this.getLastPosition())) + this.getLastPosition();
        }
    }

    public double getCurrentAng(){
        return this.getServoType().getDegFromPosition(this.getCurrentPosition());
    }

    public double getTargetPosition(){
        return this.m_TargetPosition;
    }

    public double getTargetAng(){
        return this.getServoType().getDegFromPosition(this.getTargetPosition());
    }

    public void stop(){
        if(this.isBusy()){
            this.m_LastPosition = this.getCurrentPosition();
            this.m_Speed = 0;
            this.m_ToPositionEnabled = false;
        }
    }

    public void setTargetPosition(double TargetPosition, double speed){
        if(this.isBusy()){
            this.stop();
        }
        if(TargetPosition == m_LastPosition){
            this.m_ToPositionEnabled = false;
            return;
        }
        TargetPosition = Range.clip(TargetPosition,0,1);
        speed = Range.clip(Math.abs(speed),0.0,1.0);
        this.m_TargetPosition = TargetPosition;
        this.m_EstimatedTaskTime = Math.abs((TargetPosition - this.m_LastPosition) / this.getServoType().getRevPerSec()) / speed;
        this.m_TaskTime.reset();
        this.m_Speed = speed;
        this.m_ToPositionEnabled = true;
    }

    public void setTargetAng(double TargetAng, double speed){
        this.setTargetPosition(this.getServoType().getPositionFromDeg(TargetAng),speed);
    }

    public double getTaskSpeed(){
        if(this.isBusy()){
            return this.m_Speed;
        }else{
            return 0;
        }
    }

    public double getTaskProgressRatio(){
        if(!this.isBusy()){
            return 0;
        }else{
            return Range.clip(this.m_TaskTime.seconds() / this.m_EstimatedTaskTime, 0, 1);
        }
    }

    @Override
    public boolean isBusy() {
        return this.m_ToPositionEnabled;
    }

    @Override
    public void updateStatus() {
        if(this.isBusy()){
            double taskProgress = this.getTaskProgressRatio();
            if(taskProgress >= 1){
                this.m_Servo.setPosition(m_TargetPosition);
                this.m_LastPosition = this.m_TargetPosition;
                this.m_ToPositionEnabled = false;
                this.m_Speed = 0;
                return;
            }
            this.m_Servo.setPosition(this.getCurrentPosition());
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
