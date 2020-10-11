package org.darbots.darbotsftclib.libcore.motortypes;

import org.darbots.darbotsftclib.libcore.templates.motor_related.MotorType;

public class MotorTypeImpl implements MotorType {
    private String m_MotorName;
    private double m_CountsPerRev;
    private double m_RevPerSec;
    public MotorTypeImpl(String motorName, double countsPerRev, double revPerSec){
        this.m_MotorName = motorName;
        this.m_CountsPerRev = countsPerRev;
        this.m_RevPerSec = revPerSec;
    }
    public MotorTypeImpl(MotorType motor){
        this.m_MotorName = motor.getMotorName();
        this.m_CountsPerRev = motor.getCountsPerRev();
        this.m_RevPerSec = motor.getRevPerSec();
    }
    @Override
    public String getMotorName() {
        return this.m_MotorName;
    }

    public void setMotorName(String motorName){
        this.m_MotorName = motorName;
    }

    @Override
    public double getCountsPerRev() {
        return this.m_CountsPerRev;
    }

    public void setCountsPerRev(double countsPerRev){
        this.m_CountsPerRev = countsPerRev;
    }

    @Override
    public double getRevPerSec() {
        return this.m_RevPerSec;
    }

    public void setRevPerSec(double revPerSec){
        this.m_RevPerSec = revPerSec;
    }
}
