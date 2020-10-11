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

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.darbots.darbotsftclib.libcore.runtime.GlobalRegister;
import org.darbots.darbotsftclib.libcore.templates.motor_related.MotorType;
import org.darbots.darbotsftclib.libcore.templates.motor_related.RobotMotor;


public class RobotMotorWithoutEncoder implements RobotMotor {
    protected DcMotor m_Motor;
    protected MotorType m_MotorType;
    protected ElapsedTime m_ElapsedTime;
    protected int m_CurrentCount;
    protected int m_TargetCount;
    protected MovingType m_MovingType;

    public RobotMotorWithoutEncoder(@NonNull DcMotor Motor, @NonNull MotorType MotorType){
        this.setDcMotor(Motor);
        this.m_MotorType = MotorType;
        this.m_ElapsedTime = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
        this.m_CurrentCount = 0;
        this.m_TargetCount = 0;
        this.m_MovingType = MovingType.withSpeed;
    }

    public RobotMotorWithoutEncoder(RobotMotorWithoutEncoder motor){
        this.m_Motor = motor.m_Motor;
        this.m_MotorType = motor.m_MotorType;
        this.m_CurrentCount = motor.m_CurrentCount;
        this.m_ElapsedTime = motor.m_ElapsedTime;
        this.m_CurrentCount = motor.m_CurrentCount;
        this.m_TargetCount = motor.m_TargetCount;
        this.m_MovingType = motor.m_MovingType;
    }

    public DcMotor getDcMotor() {
        return this.m_Motor;
    }

    public void setDcMotor(@NonNull DcMotor Motor) {
        this.m_Motor = Motor;
    }

    @Override
    public MotorType getMotorType() {
        return this.m_MotorType;
    }

    @Override
    public void setMotorType(@NonNull MotorType MotorType) {
        this.updateCount();
        this.m_MotorType = MotorType;
    }

    @Override
    public int getCurrentCount() {
        this.updateCount();
        return this.m_CurrentCount;
    }

    @Override
    public int getTargetCount() {
        return this.m_TargetCount;
    }

    @Override
    public void setTargetCount(int Count) {
        this.m_TargetCount = Count;
        if(this.m_MovingType == MovingType.toCount){
            this.setToPosition();
        }
    }

    @Override
    public double getPower() {
        return m_Motor.getPower();
    }

    @Override
    public void setPower(double Pwr) {
        this.updateCount();
        m_Motor.setPower(Pwr);
        if(this.m_MovingType == MovingType.toCount){
            this.setToPosition();
        }
    }

    @Override
    public boolean isDirectionReversed() {
        return m_Motor.getDirection() == DcMotorSimple.Direction.REVERSE;
    }

    @Override
    public void setDirectionReversed(boolean isReversed) {
        if(isReversed){
            m_Motor.setDirection(DcMotorSimple.Direction.REVERSE);
        }else{
            m_Motor.setDirection(DcMotorSimple.Direction.FORWARD);
        }
    }

    @Override
    public DcMotor.ZeroPowerBehavior getZeroPowerBehavior() {
        return m_Motor.getZeroPowerBehavior();
    }

    @Override
    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior ZeroPwrBehavior) {
        m_Motor.setZeroPowerBehavior(ZeroPwrBehavior);
    }

    @Override
    public MovingType getCurrentMovingType() {
        if(this.m_MovingType == MovingType.toCount){
            return MovingType.toCount;
        }else if(this.m_MovingType == MovingType.withSpeed){
            return MovingType.withSpeed;
        }else{
            return MovingType.reset;
        }
    }

    @Override
    public void setCurrentMovingType(MovingType movingType) {
        if(movingType == MovingType.toCount){
            this.setToPosition();
        }else if(movingType == MovingType.withSpeed){
            m_Motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            this.m_MovingType = MovingType.withSpeed;
        }else{ // if(movingType == MovingType.reset){
            this.resetEncoder();
        }
    }

    protected void resetEncoder(){
        this.setPower(0);
        this.m_CurrentCount = 0;
        this.m_TargetCount = 0;
        this.m_MovingType = MovingType.withSpeed;
    }

    protected void setToPosition(){
        this.m_MovingType = MovingType.toCount;
        this.updateCount();
        double pwr = Math.abs(m_Motor.getPower());
        if(this.m_TargetCount < this.m_CurrentCount){
            pwr = -pwr;
            this.m_Motor.setPower(pwr);
        }else if(this.m_TargetCount == this.m_CurrentCount){
            this.m_Motor.setPower(0);
        }else{ //this.m_TargetCount > this.m_CurrentCount
            this.m_Motor.setPower(pwr);
        }
    }

    @Override
    public boolean isBusy() {
        return m_Motor.getPower() != 0;
    }

    @Override
    public void updateStatus() {
        if(this.m_MovingType == MovingType.toCount){
            if(m_Motor.getPower() > 0){
                updateCount();
                if(this.m_CurrentCount >= this.m_TargetCount){
                    this.m_Motor.setPower(0);
                }
            }else{ //(m_Motor.getPower() < 1){
                updateCount();
                if(this.m_CurrentCount <= this.m_TargetCount){
                    this.m_Motor.setPower(0);
                }
            }
        }else{
            updateCount();
        }
    }

    protected void updateCount(){
        this.m_CurrentCount += this.getPower() * this.getMotorType().getCountsPerRev() * this.getMotorType().getRevPerSec() * this.m_ElapsedTime.seconds();
        this.m_ElapsedTime.reset();
    }

    @Override
    public void waitUntilFinish() {
        if(this.getCurrentMovingType() == MovingType.toCount){
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
    @Override
    public String getMotorStatusString(){
        String result = "DeviceType: RobotMotorWithoutEncoder, ";
        result += "DeviceName: " + this.getDcMotor().getDeviceName() + ", ";
        result += "RunningState: " + this.getCurrentMovingType().name() + ", ";
        result += "CurrentCount: " + this.getCurrentCount() + ", ";
        result += "TargetCount: " + this.getTargetCount() + ", ";
        result += "Power: " + this.getPower();
        return result;
    }
}
