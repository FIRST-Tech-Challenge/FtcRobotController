package org.firstinspires.ftc.teamcode.main.utils.autonomous.sensors.distance.wrappers;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class DistanceSensorWrapper implements SensorWrapper {

    DistanceUnit units = DistanceUnit.CM;
    DistanceSensor sensor;
    HardwareMap hardwareMap;
    String name;


    public DistanceSensorWrapper(HardwareMap hardwareMap, String name) {
        this.hardwareMap = hardwareMap;
        this.name = name;

        sensor = hardwareMap.get(DistanceSensor.class, name);
        sensor.resetDeviceConfigurationForOpMode();
    }

    @Override
    public void setUnits(DistanceUnit unit) {
        units = unit;
    }

    @Override
    public int getData() {
        return (int) sensor.getDistance(units);
    }

    @Override
    public String getName() {
        return name;
    }

    @Override
    public boolean didTimeoutOccur() {
        try {
            return ((Rev2mDistanceSensor) sensor).didTimeoutOccur();
        } catch(ClassCastException e) {
            return false;
        }
    }

    public DistanceSensor getSensor() {
        return sensor;
    }

}
