package org.firstinspires.ftc.teamcode.main.autonomous.sensors.distance;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.competition.utils.devices.EyeSquaredSeaDistanceSensor;
import org.firstinspires.ftc.teamcode.main.autonomous.sensors.SensorWrapper;

public class UltrasonicDistanceSensor implements SensorWrapper {
    public EyeSquaredSeaDistanceSensor sensor;
    public DistanceUnit units = DistanceUnit.CM;
    public String name;

    public UltrasonicDistanceSensor(HardwareMap hardwareMap, String name) {
        this.name = name;
        sensor = hardwareMap.get(EyeSquaredSeaDistanceSensor.class, name);
    }

    @Override
    public void setUnits(DistanceUnit unit) {
        units = unit;
    }

    @Override
    public double getData() {
        sensor.writeData(EyeSquaredSeaDistanceSensor.Commands.INITIATE_WRITE, (short) EyeSquaredSeaDistanceSensor.Commands.WRITE_RANGE_COMMAND.bVal);
        return sensor.readData(EyeSquaredSeaDistanceSensor.Commands.READ_LAST);
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
