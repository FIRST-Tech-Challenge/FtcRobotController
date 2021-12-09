package org.firstinspires.ftc.teamcode.competition.utils.interactions.items;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.competition.utils.interactions.InteractionSurface;

public class StandardDistanceSensor extends InteractionSurface {

    private final DistanceSensor SENSOR;

    public StandardDistanceSensor(HardwareMap hardware, String name) {
        SENSOR = hardware.get(DistanceSensor.class, name);
    }

    public double getDistance(DistanceUnit unit) {
        return SENSOR.getDistance(unit);
    }

    public void stop() {
        close();
    }

    public void close() {
        SENSOR.close();
    }

    public DistanceSensor getInternalSensor() {
        return SENSOR;
    }

    @Override
    public boolean isInputDevice() {
        return false;
    }

    @Override
    public boolean isOutputDevice() {
        return true;
    }

}
