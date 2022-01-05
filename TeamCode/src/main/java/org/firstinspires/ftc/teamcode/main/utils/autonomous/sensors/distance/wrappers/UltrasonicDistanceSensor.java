package org.firstinspires.ftc.teamcode.main.utils.autonomous.sensors.distance.wrappers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.main.utils.devices.EyeSquaredSeaDistanceSensor;

public class UltrasonicDistanceSensor implements SensorWrapper {
    public EyeSquaredSeaDistanceSensor sensor;
    public DistanceUnit units = DistanceUnit.CM;
    public String name;

    public UltrasonicDistanceSensor(HardwareMap hardwareMap, String name) {
        this.name = name;
        sensor = hardwareMap.get(EyeSquaredSeaDistanceSensor.class, name);
        sensor.resetDeviceConfigurationForOpMode();
    }

    public UltrasonicDistanceSensor(HardwareMap hardwareMap, String name, LinearOpMode opMode) {
        this.name = name;
        sensor = hardwareMap.get(EyeSquaredSeaDistanceSensor.class, name);
    }

    @Override
    public void setUnits(DistanceUnit unit) {
        units = unit;
    }

    @Override
    public synchronized int getData() {
        sensor.writeData(EyeSquaredSeaDistanceSensor.Commands.WRITE_RANGE_COMMAND);
        try {
            wait(80L);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        return (int) sensor.readData(EyeSquaredSeaDistanceSensor.Commands.READ_LAST.bVal);
    }

    @Override
    public String getName() {
        return name;
    }

    @Override
    public boolean didTimeoutOccur() {
        return false;
    }

    public EyeSquaredSeaDistanceSensor getSensor() {
        return sensor;
    }

}
