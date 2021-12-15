package org.firstinspires.ftc.teamcode.competition.utils.interactions.items;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.competition.utils.interactions.InteractionSurface;
import org.firstinspires.ftc.teamcode.main.autonomous.sensors.distance.DistanceSensorWrapper;
import org.firstinspires.ftc.teamcode.main.autonomous.sensors.distance.UltrasonicDistanceSensor;

public class StandardUltrasonicDistanceSensor extends InteractionSurface {

    private final UltrasonicDistanceSensor SENSOR;

    public StandardUltrasonicDistanceSensor(HardwareMap hardware, String name) {
        SENSOR = new UltrasonicDistanceSensor(hardware, name);
    }

    public double getDistance(DistanceUnit unit) {
        SENSOR.setUnits(unit);
        return SENSOR.getData();
    }

    public void stop() {
        close();
    }

    public void close() {
        SENSOR.getSensor().close();
    }

    public UltrasonicDistanceSensor getInternalSensor() {
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
