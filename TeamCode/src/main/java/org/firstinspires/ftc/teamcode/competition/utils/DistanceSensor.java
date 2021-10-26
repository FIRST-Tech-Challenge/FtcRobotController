package org.firstinspires.ftc.teamcode.competition.utils;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Simple implementation of Qualcomm's DistanceSensor interface allowing for creation of a servo with a constructor.
 * @author Thomas Ricci
 */
public class DistanceSensor implements com.qualcomm.robotcore.hardware.DistanceSensor {

    private final com.qualcomm.robotcore.hardware.DistanceSensor INTERNAL_DISTANCE_SENSOR;

    /**
     * Creates a distance sensor
     * @param hardware The hardware map the sensor's associated with
     * @param name The name of the sensor to represent
     */
    public DistanceSensor(HardwareMap hardware, String name) {
        INTERNAL_DISTANCE_SENSOR = hardware.get(com.qualcomm.robotcore.hardware.DistanceSensor.class, name);
    }

    @Override
    public double getDistance(DistanceUnit unit) {
        return INTERNAL_DISTANCE_SENSOR.getDistance(unit);
    }

    @Override
    public Manufacturer getManufacturer() {
        return INTERNAL_DISTANCE_SENSOR.getManufacturer();
    }

    @Override
    public String getDeviceName() {
        return INTERNAL_DISTANCE_SENSOR.getDeviceName();
    }

    @Override
    public String getConnectionInfo() {
        return INTERNAL_DISTANCE_SENSOR.getConnectionInfo();
    }

    @Override
    public int getVersion() {
        return INTERNAL_DISTANCE_SENSOR.getVersion();
    }

    @Override
    public void resetDeviceConfigurationForOpMode() {
        INTERNAL_DISTANCE_SENSOR.resetDeviceConfigurationForOpMode();
    }

    @Override
    public void close() {
        INTERNAL_DISTANCE_SENSOR.close();
    }
}
