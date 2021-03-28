package com.technototes.library.hardware.sensor;

import com.technototes.logger.Log;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/** Class for range sensors
 * @author Alex Stedman
 */
public class RangeSensor extends Sensor<com.qualcomm.robotcore.hardware.DistanceSensor> {

    private DistanceUnit distanceUnit;

    /** Create a range sensor
     *
     * @param device The sensor device
     */
    public RangeSensor(com.qualcomm.robotcore.hardware.DistanceSensor device) {
        super(device);
    }

    /** Create a range sensor
     *
     * @param deviceName The device name
     */
    public RangeSensor(String deviceName) {
        super(deviceName);
    }

    @Override
    public double getSensorValue() {
        return getDevice().getDistance(distanceUnit);
    }

    /** Get the value with a specified distance Unit
     *
     * @param distanceUnit The unit
     * @return The distance
     */
    public double getSensorValue(DistanceUnit distanceUnit) {
        return getDevice().getDistance(distanceUnit);
    }

    /** Get the current distance unit
     *
     * @return The distance unit
     */
    public DistanceUnit getDistanceUnit() {
        return distanceUnit;
    }

    /** Set the distance unit
     *
     * @param distanceUnit The unit
     * @return This
     */
    public RangeSensor setDistanceUnit(DistanceUnit distanceUnit) {
        this.distanceUnit = distanceUnit;
        return this;
    }
}
