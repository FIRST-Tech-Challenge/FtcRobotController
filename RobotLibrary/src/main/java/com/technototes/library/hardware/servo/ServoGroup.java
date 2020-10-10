package com.technototes.library.hardware.servo;

import com.technototes.library.hardware.HardwareDeviceGroup;
import com.technototes.logger.Log;

public class ServoGroup extends Servo implements HardwareDeviceGroup<Servo> {
    private Servo[] followers;

    public ServoGroup(Servo leader, Servo... f) {
        super(leader);
        followers = f;
        for (Servo s : f) {
            s.follow(leader);
        }
    }

    @Override
    public Servo[] getFollowers() {
        return followers;
    }

    @Override
    public Servo[] getAllDevices() {
        Servo[] m = new Servo[followers.length + 1];
        m[0] = this;
        for (int i = 1; i < m.length; i++) {
            m[i] = followers[i - 1];
        }
        return m;
    }
    @Log
    @Override
    public double getSensorValue() {
        return super.getSensorValue();
    }
}