package org.firstinspires.ftc.teamcode.robots.csbot.simulation;

import com.qualcomm.robotcore.hardware.VoltageSensor;

public class VoltageSensorSim implements VoltageSensor {
    @Override
    public double getVoltage() {
        return 12;
    }

    @Override
    public Manufacturer getManufacturer() {
        return null;
    }

    @Override
    public String getDeviceName() {
        return null;
    }

    @Override
    public String getConnectionInfo() {
        return null;
    }

    @Override
    public int getVersion() {
        return 0;
    }

    @Override
    public void resetDeviceConfigurationForOpMode() {

    }

    @Override
    public void close() {

    }
}
