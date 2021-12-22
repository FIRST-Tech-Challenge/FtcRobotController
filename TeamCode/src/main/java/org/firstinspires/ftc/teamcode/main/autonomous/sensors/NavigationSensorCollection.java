package org.firstinspires.ftc.teamcode.main.autonomous.sensors;

import org.firstinspires.ftc.teamcode.competition.utils.interactions.items.StandardDistanceSensor;
import org.firstinspires.ftc.teamcode.competition.utils.interactions.items.StandardIMU;
import org.firstinspires.ftc.teamcode.main.autonomous.location.pipeline.PositionSystem;

public class NavigationSensorCollection {
    public StandardDistanceSensor north;
    public StandardDistanceSensor east;
    public StandardDistanceSensor west;

    public StandardIMU imu;

    public NavigationSensorCollection(StandardDistanceSensor north,
                                      StandardDistanceSensor east,
                                      StandardDistanceSensor west,
                                      StandardIMU imu) {
        this.north = north;
        this.east = east;
        this.west = west;
        this.imu = imu;
    }
}
