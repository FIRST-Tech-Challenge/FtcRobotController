package org.firstinspires.ftc.teamcode.competition.utils.interactions.items;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.competition.utils.interactions.InteractionSurface;
import org.firstinspires.ftc.teamcode.main.autonomous.sensors.SensorWrapper;
import org.firstinspires.ftc.teamcode.main.autonomous.sensors.distance.DistanceSensorWrapper;
import org.firstinspires.ftc.teamcode.main.autonomous.sensors.distance.UltrasonicDistanceSensor;

/**
 * A StandardDistanceSensor represents a DistanceSensorWrapper and/or an UltrasonicDistanceSensor, depending on the robot's configuration.
 */
public class StandardDistanceSensor extends InteractionSurface {

    private DistanceSensorWrapper SENSOR_DISTANCE;
    private UltrasonicDistanceSensor SENSOR_ULTRASONIC;

    public StandardDistanceSensor(HardwareMap hardware, String name) {
        try {
            SENSOR_ULTRASONIC = new UltrasonicDistanceSensor(hardware, name);
        } catch(Exception e) {
            SENSOR_ULTRASONIC = null;
        }
        try {
            SENSOR_DISTANCE = new DistanceSensorWrapper(hardware, name);
        } catch(Exception e) {
            SENSOR_DISTANCE = null;
        }
    }

    public int getData(DistanceUnit unit) {
        return getDistance(unit);
    }

    public int getDistance(DistanceUnit unit) {
        try {
            SENSOR_DISTANCE.setUnits(unit);
            return (int) SENSOR_DISTANCE.getData();
        } catch(Exception ignored) {
            try {
                SENSOR_ULTRASONIC.setUnits(unit);
                return (int) SENSOR_ULTRASONIC.getData();
            } catch(Exception ignoredAlso) {
                return 0;
            }
        }
    }

    public void stop() {
        close();
    }

    public void close() {
        try {
            SENSOR_DISTANCE.getSensor().close();
        } catch(Exception ignored) {
            try {
                SENSOR_ULTRASONIC.getSensor().close();
            } catch(Exception ignored1) {}
        }
    }

    public enum StandardDistanceSensorInternalType {
        DEFAULT,
        ULTRASONIC
    }

    public SensorWrapper getInternalSensor(StandardDistanceSensorInternalType type) {
        if(type == StandardDistanceSensorInternalType.DEFAULT) {
            return SENSOR_DISTANCE;
        }else{
            return SENSOR_ULTRASONIC;
        }
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
