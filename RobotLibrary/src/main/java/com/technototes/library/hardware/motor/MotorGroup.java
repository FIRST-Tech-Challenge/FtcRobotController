package com.technototes.library.hardware.motor;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.technototes.library.hardware.HardwareDeviceGroup;

public class MotorGroup<T extends Motor> extends Motor<DcMotor> implements HardwareDeviceGroup<Motor> {
    private Motor[] followers;

    public MotorGroup(Motor leader, Motor... f) {
        super(leader);
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


}
