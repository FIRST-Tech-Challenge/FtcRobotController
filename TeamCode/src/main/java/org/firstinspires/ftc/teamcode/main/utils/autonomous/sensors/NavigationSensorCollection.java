package org.firstinspires.ftc.teamcode.main.utils.autonomous.sensors;

import org.firstinspires.ftc.teamcode.main.utils.interactions.items.StandardDistanceSensor;
import org.firstinspires.ftc.teamcode.main.utils.interactions.items.StandardIMU;

public class NavigationSensorCollection {
    public StandardDistanceSensor north;
    public StandardDistanceSensor east;
    public StandardDistanceSensor west;

    public StandardIMU imu;
    public int imuOffset;

    public NavigationSensorCollection(StandardDistanceSensor north,
                                      StandardDistanceSensor east,
                                      StandardDistanceSensor west,
                                      StandardIMU imu,
                                      int imuOffset) {
        this.north = north;
        this.east = east;
        this.west = west;
        this.imu = imu;
        this.imuOffset = imuOffset;
    }
}
