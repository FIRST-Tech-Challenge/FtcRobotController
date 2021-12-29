package org.firstinspires.ftc.teamcode.main.utils.interactions.items;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.main.utils.interactions.InteractionSurface;

public class StandardTouchSensor extends InteractionItem {

    private final TouchSensor SENSOR;

    /**
     * Creates a new Touch Sensor.
     * @param hardware The hardware map to get the physical sensor from
     * @param name The name of the sensor
     */
    public StandardTouchSensor(HardwareMap hardware, String name) {
        SENSOR = hardware.get(TouchSensor.class, name);
        SENSOR.resetDeviceConfigurationForOpMode();
    }

    /**
     * Checks if the sensor is pressed
     * @return Returns true if pressed
     */
    public boolean isPressed() {
        return SENSOR.isPressed();
    }

    /**
     * Represents how much force is applied to the touch sensor; for some touch sensors this value will only ever be 0 or 100.
     * @return An integer between 0 and 100
     */
    public int getForce() {
        return (int) Range.clip(SENSOR.getValue() * 100, 0, 100);
    }

    public void stop() {}

    public TouchSensor getInternalSensor() {
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
