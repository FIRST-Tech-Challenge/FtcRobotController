package org.firstinspires.ftc.teamcode.main.utils.interactions.items;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.main.utils.interactions.InteractionSurface;
import org.firstinspires.ftc.teamcode.main.utils.autonomous.sensors.distance.wrappers.SensorWrapper;

@Deprecated
public abstract class StandardSensor extends InteractionItem {
    public abstract int getData();

    abstract int getDistance(DistanceUnit unit);

    abstract void close();

    abstract SensorWrapper getInternalSensor(StandardDistanceSensor.StandardDistanceSensorInternalType type);

    public abstract boolean didTimeoutOccur();
}
