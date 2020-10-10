package com.technototes.library.hardware.sensor;

import com.technototes.library.hardware.HardwareDevice;
import com.technototes.library.hardware.Sensored;

public abstract class Sensor<T extends com.qualcomm.robotcore.hardware.HardwareDevice> extends HardwareDevice<T> implements Sensored {
    public Sensor(T d) {
        super(d);
    }

    public Sensor(String s) {
        super(s);
    }

}
