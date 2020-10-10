package com.technototes.library.hardware.sensor;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.technototes.logger.Log;

public class DigitalSensor extends Sensor<DigitalChannel> {
    public DigitalSensor(DigitalChannel d) {
        super(d);
    }

    public DigitalSensor(String s) {
        super(s);
    }

    @Log
    @Override
    public double getSensorValue() {
        return device.getState() ? 1 : 0;
    }

    public boolean getSensorValueAsBoolean() {
        return device.getState();
    }
}
