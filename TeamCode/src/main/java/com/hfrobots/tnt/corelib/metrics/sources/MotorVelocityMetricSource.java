package com.hfrobots.tnt.corelib.metrics.sources;

import com.hfrobots.tnt.corelib.metrics.GaugeMetricSource;
import com.hfrobots.tnt.corelib.metrics.MetricsSampler;
import com.hfrobots.tnt.util.NamedDeviceMap;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import lombok.EqualsAndHashCode;
import lombok.NonNull;

@EqualsAndHashCode
public class MotorVelocityMetricSource implements GaugeMetricSource {
    private final DcMotorEx motor;

    private final String sampleName;

    private int lastEncoderValue;

    private long lastValueTimestamp;

    public MotorVelocityMetricSource(@NonNull final NamedDeviceMap.NamedDevice<DcMotorEx> namedMotor) {
        this.motor = namedMotor.getDevice();

        sampleName = String.format("dcm_vel_%s", namedMotor.getName());
        lastEncoderValue = motor.getCurrentPosition();
        lastValueTimestamp = System.currentTimeMillis();
    }

    @Override
    public String getSampleName() {
        return sampleName;
    }

    @Override
    public double getValue() {
        int newEncoderValue = motor.getCurrentPosition();
        double deltaEncoderPosition = newEncoderValue - lastEncoderValue;
        lastEncoderValue = newEncoderValue;

        long now = System.currentTimeMillis();
        double deltaTimeMillis = now - lastValueTimestamp;
        lastValueTimestamp = now;

        if (deltaTimeMillis == 0) {
            return MetricsSampler.NO_REPORT_VALUE;
        }

        return (deltaEncoderPosition / deltaTimeMillis) * 1000;
    }
}
