package org.firstinspires.ftc.teamcode.competition.utils.interactions.items;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.competition.utils.interactions.InteractionSurface;
import org.firstinspires.ftc.teamcode.main.autonomous.sensors.SensorWrapper;
import org.firstinspires.ftc.teamcode.main.autonomous.sensors.distance.DistanceSensorWrapper;

public class StandardDistanceSensor extends InteractionSurface {

    private final DistanceSensorWrapper SENSOR;

    public StandardDistanceSensor(HardwareMap hardware, String name) {
        SENSOR = new DistanceSensorWrapper(hardware, name);
    }

    public int getData(DistanceUnit unit) {
        return getDistance(unit);
    }

    public int getDistance(DistanceUnit unit) {
        SENSOR.setUnits(unit);
        return SENSOR.getData();
    }

    public void stop() {
        close();
    }

    public void close() {
        SENSOR.getSensor().close();
    }

    public DistanceSensorWrapper getInternalSensor() {
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
