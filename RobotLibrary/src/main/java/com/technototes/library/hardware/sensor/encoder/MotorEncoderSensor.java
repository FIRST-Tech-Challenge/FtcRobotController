package com.technototes.library.hardware.sensor.encoder;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.technototes.library.hardware.sensor.Sensor;
import com.technototes.library.measurement.unit.MotorEncoderUnit;
import com.technototes.logger.Log;

import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;

public class MotorEncoderSensor extends Sensor<DcMotor> implements Encoder {

    private double zero = 0;
    private DoubleSupplier supplier;
    public MotorEncoderUnit unit;
    public MotorEncoderSensor(DcMotor d, MotorEncoderUnit m) {
        super(d);
        supplier = () -> d.getCurrentPosition();
        unit = m;
        unit.setSupplier(supplier);
        zeroEncoder();
    }

    @Override
    public void zeroEncoder() {
        zero = unit.get();
    }

    @Log
    @Override
    public double getSensorValue() {
        return unit.get()-zero;
    }
}
