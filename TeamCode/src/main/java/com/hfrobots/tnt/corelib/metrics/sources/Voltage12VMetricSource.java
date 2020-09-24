package com.hfrobots.tnt.corelib.metrics.sources;

import com.hfrobots.tnt.corelib.metrics.GaugeMetricSource;
import com.qualcomm.hardware.lynx.LynxModule;

import org.firstinspires.ftc.robotcore.external.navigation.VoltageUnit;

import lombok.EqualsAndHashCode;

@EqualsAndHashCode
public class Voltage12VMetricSource implements GaugeMetricSource {

    private final LynxModule lynxModule;

    private final String name;

    public Voltage12VMetricSource(LynxModule lynxModule) {
        this.lynxModule = lynxModule;
        int moduleAddress = lynxModule.getModuleAddress();

        name = String.format("hub_%d_12V", moduleAddress);
    }

    @Override
    public String getSampleName() {
        return name;
    }

    @Override
    public double getValue() {
        return lynxModule.getInputVoltage(VoltageUnit.VOLTS);
    }
}
