package org.firstinspires.ftc.teamcode.robots.csbot.simulation;

import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class DistanceSensorSim implements DistanceSensor {

    private double distance;

    public DistanceSensorSim(double distance) {
        this.distance = distance;
    }

    @Override
    public double getDistance(DistanceUnit unit) {
        return distance;
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
