package org.firstinspires.ftc.teamcode.main.autonomous.sensors;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public interface SensorWrapper {
    DistanceUnit units = DistanceUnit.CM;

    void setUnits(DistanceUnit unit);
    double getData();
    String getName();
    boolean didTimeoutOccur();
}

