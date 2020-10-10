package com.technototes.library.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;

public abstract class HardwareDevice<T extends com.qualcomm.robotcore.hardware.HardwareDevice> {
    public static HardwareMap hardwareMap = null;
    public T device;

    public HardwareDevice(T d) {
        device = d;
    }

    public HardwareDevice(HardwareDevice<T> d) {
        this(d.getDevice());
    }

    public HardwareDevice(String s) {
        this(hardwareMap.get((Class<T>) com.qualcomm.robotcore.hardware.HardwareDevice.class, s));
    }

    public T getDevice() {
        return device;
    }
}
