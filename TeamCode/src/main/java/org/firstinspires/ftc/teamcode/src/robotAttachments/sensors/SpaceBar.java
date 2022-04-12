package org.firstinspires.ftc.teamcode.src.robotAttachments.sensors;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.jetbrains.annotations.NotNull;

public class SpaceBar implements TouchSensor {
    private final TouchSensor sensor1;

    private final TouchSensor sensor2;


    public SpaceBar(final @NotNull HardwareMap hardwareMap, final @NotNull String sensor1Name, final @NotNull String sensor2Name) {
        sensor1 = hardwareMap.touchSensor.get(sensor1Name);
        sensor2 = hardwareMap.touchSensor.get(sensor2Name);
    }

    @Override
    public double getValue() {
        return (sensor1.getValue() + sensor2.getValue()) / 2;
    }

    @Override
    public boolean isPressed() {
        return sensor1.isPressed() || sensor2.isPressed();
    }

    @Override
    public Manufacturer getManufacturer() {
        return Manufacturer.Unknown;
    }

    @Override
    public String getDeviceName() {
        return sensor1.getDeviceName() + " and " + sensor2.getDeviceName();
    }

    @Override
    public String getConnectionInfo() {
        return sensor1.getConnectionInfo() + " and " + sensor2.getConnectionInfo();
    }

    @Override
    public int getVersion() {
        return 0;
    }

    @Override
    public void resetDeviceConfigurationForOpMode() {
        sensor2.resetDeviceConfigurationForOpMode();
        sensor1.resetDeviceConfigurationForOpMode();
    }

    @Override
    public void close() {
        sensor2.close();
        sensor1.close();
    }

}
