package org.darbots.darbotsftclib.libcore.templates.servo_related;

import org.darbots.darbotsftclib.libcore.templates.motor_related.MotorType;

public abstract class ServoType implements MotorType {
    public abstract String getServoName();
    public String getMotorName(){
        return getServoName();
    }
    @Override
    public double getCountsPerRev(){
        return Math.abs(getMaxDeg() - getMinDeg());
    }
    public abstract double getMaxDeg();
    public abstract double getMinDeg();
    public double getDegFromPosition(double Position){
        return Position * getCountsPerRev() + getMinDeg();
    }
    public double getPositionFromDeg(double Deg){
        return (Deg - getMinDeg()) / getCountsPerRev();
    }
    public abstract double getPulseLowerInMicroSeconds();
    public abstract double getPulseUpperInMicroSeconds();
}
