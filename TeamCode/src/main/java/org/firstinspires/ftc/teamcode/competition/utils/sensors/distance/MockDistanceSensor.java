package org.firstinspires.ftc.teamcode.competition.utils.sensors.distance;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.competition.utils.sensors.SensorWrapper;

public class MockDistanceSensor implements SensorWrapper {

    MockDistanceSensorData data = new MockDistanceSensorData(10, false, "Mock Sensor 1");

    @Override
    public void setUnits(DistanceUnit unit) { }

    @Override
    public double getData() {
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

