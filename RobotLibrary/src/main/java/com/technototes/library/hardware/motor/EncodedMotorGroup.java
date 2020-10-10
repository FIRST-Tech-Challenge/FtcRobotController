package com.technototes.library.hardware.motor;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.technototes.library.hardware.HardwareDeviceGroup;
import com.technototes.logger.Log;


public class EncodedMotorGroup extends EncodedMotor<DcMotor> implements HardwareDeviceGroup<Motor> {
    private Motor[] followers;

    public EncodedMotorGroup(EncodedMotor leader, Motor... f) {
        super(leader, leader.encoder.unit);
        followers = f;
        for (Motor s : f) {
            s.follow(leader);
        }
    }

    @Override
    public Motor[] getFollowers() {
        return followers;
    }

    @Override
    public Motor[] getAllDevices() {
        Motor[] m = new Motor[followers.length + 1];
        m[0] = this;
        for (int i = 1; i < m.length; i++) {
            m[i] = followers[i - 1];
        }
        return m;
    }
    @Override
    @Log
    public double getSpeed() {
        return super.getSpeed();
    }
}
