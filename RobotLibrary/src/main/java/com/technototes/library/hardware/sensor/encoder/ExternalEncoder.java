package com.technototes.library.hardware.sensor.encoder;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.technototes.library.hardware.sensor.Sensor;

public class ExternalEncoder extends Sensor<AnalogInput> implements Encoder {
    private double zero = 0;

    public ExternalEncoder(AnalogInput device) {
        super(device);
    }
    public ExternalEncoder(String deviceName) {
        super(deviceName);
    }


    @Override
    public void zeroEncoder() {
        zero = getDevice().getVoltage();
    }

    @Override
    public double getSensorValue() {
        return getDevice().getVoltage()-zero;
    }
}
