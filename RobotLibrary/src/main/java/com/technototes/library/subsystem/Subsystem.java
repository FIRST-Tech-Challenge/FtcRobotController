package com.technototes.library.subsystem;

import com.technototes.library.hardware.HardwareDevice;

public abstract class Subsystem<T extends HardwareDevice> {
    public T[] devices;

    public Subsystem(T... d) {
        devices = d;
    }

    public T[] getDevices() {
        return devices;
    }
}
