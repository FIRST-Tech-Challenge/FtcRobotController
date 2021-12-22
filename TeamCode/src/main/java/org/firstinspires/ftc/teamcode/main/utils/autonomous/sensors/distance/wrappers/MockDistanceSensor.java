package org.firstinspires.ftc.teamcode.main.utils.autonomous.sensors.distance.wrappers;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class MockDistanceSensor implements SensorWrapper {

    MockDistanceSensorData data = new MockDistanceSensorData(0, false, "Mock Sensor 1");

    @Override
    public void setUnits(DistanceUnit unit) { }

    @Override
    public int getData() {
        return data.getDistance();
    }

    @Override
    public String getName() {
        return data.getName();
    }

    @Override
    public boolean didTimeoutOccur() {
        return data.isDidTimeoutOccur();
    }
}

