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

import org.darbots.darbotsftclib.libcore.runtime.GlobalRegister;
import org.darbots.darbotsftclib.libcore.templates.motor_related.MotorType;
import org.darbots.darbotsftclib.libcore.templates.motor_related.RobotMotor;


public class RobotMotorWithEncoder implements RobotMotor {
    protected DcMotor m_Motor;
    protected MotorType m_MotorType;

    public RobotMotorWithEncoder(@NonNull DcMotor Motor, @NonNull MotorType MotorType){
        this.m_Motor = Motor;
        this.m_MotorType = MotorType;
    }

    public RobotMotorWithEncoder(RobotMotorWithEncoder motor){
        this.m_Motor = motor.m_Motor;
        this.m_MotorType = motor.m_MotorType;
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
        this.m_MotorType = MotorType;
    }

    @Override
    public int getCurrentCount() {
        return m_Motor.getCurrentPosition();
    }

    @Override
    public int getTargetCount() {
        return m_Motor.getTargetPosition();
    }

    @Override
    public void setTargetCount(int Count) {
        m_Motor.setTargetPosition(Count);
    }

    @Override
    public double getPower() {
        return m_Motor.getPower();
    }

    @Override
    public void setPower(double Pwr) {
        m_Motor.setPower(Pwr);
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
        if(m_Motor.getMode() == DcMotor.RunMode.RUN_TO_POSITION){
            return MovingType.toCount;
        }else if(m_Motor.getMode() == DcMotor.RunMode.RUN_USING_ENCODER || m_Motor.getMode() == DcMotor.RunMode.RUN_WITHOUT_ENCODER){
            return MovingType.withSpeed;
        }else{
            return MovingType.reset;
        }
    }

    @Override
    public void setCurrentMovingType(MovingType movingType) {
        if(movingType == MovingType.toCount){
            m_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }else if(movingType == MovingType.withSpeed){
            m_Motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }else{ // if(movingType == MovingType.reset){
            m_Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m_Motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            this.setPower(0);
        }
    }

    @Override
    public boolean isBusy() {
        return m_Motor.isBusy();
    }

    @Override
    public void updateStatus() {
        return;
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
            }
        }else{
            return;
        }
    }
    @Override
    public String getMotorStatusString(){
        String result = "DeviceType: RobotMotorWithEncoder, ";
        result += "DeviceName: " + this.getDcMotor().getDeviceName() + ", ";
        result += "RunningState: " + this.getCurrentMovingType().name() + ", ";
        result += "CurrentCount: " + this.getCurrentCount() + ", ";
        result += "TargetCount: " + this.getTargetCount() + ", ";
        result += "Power: " + this.getPower();
        return result;
    }
}
