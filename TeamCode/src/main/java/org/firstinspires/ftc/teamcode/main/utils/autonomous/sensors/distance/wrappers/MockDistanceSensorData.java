package org.firstinspires.ftc.teamcode.main.utils.autonomous.sensors.distance.wrappers;

public class MockDistanceSensorData {
    int Distance;
    boolean didTimeoutOccur;
    String name = "Mock Sensor";

    public MockDistanceSensorData(int distance, boolean didTimeoutOccur, String name) {
        Distance = distance;
        this.didTimeoutOccur = didTimeoutOccur;
        this.name = name;
    }

    public MockDistanceSensorData() {
    }

    public int getDistance() {
        return Distance;
    }

    public void setDistance(int distance) {
        Distance = distance;
    }

    public boolean isDidTimeoutOccur() {
        return didTimeoutOccur;
    }

    public void setDidTimeoutOccur(boolean didTimeoutOccur) {
        this.didTimeoutOccur = didTimeoutOccur;
    }

    public String getName() {
        return name;
    }

    public void setName(String name) {
        this.name = name;
    }
}
