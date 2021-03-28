package com.technototes.library.hardware.sensor;

import com.technototes.library.hardware.HardwareDevice;
import com.technototes.library.hardware.Sensored;

/** Root class for sensors
 * @author Alex Stedman
 * @param <T> The Sensor hardware device
 */
public abstract class Sensor<T extends com.qualcomm.robotcore.hardware.HardwareDevice> extends HardwareDevice<T> implements Sensored {
    /** Crease sensor
     *
     * @param device The device
     */
    public Sensor(T device) {
        super(device);
    }

    /** Create sensor
     *
     * @param deviceName The device name in hardware map
     */
    public Sensor(String deviceName) {
        super(deviceName);
    }

}
