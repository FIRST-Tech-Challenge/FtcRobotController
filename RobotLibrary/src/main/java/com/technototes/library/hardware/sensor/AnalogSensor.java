package com.technototes.library.hardware.sensor;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.technototes.logger.Log;

/** Class for analog sensors
 * @author Alex Stedman
 */
public class AnalogSensor extends Sensor<AnalogInput> {
    /** Make an analog sensor
     *
     * @param device The analog device
     */
    public AnalogSensor(AnalogInput device) {
        super(device);
    }

    /** Make an analog sensor
     *
     * @param deviceName The device name in hardware map
     */
    public AnalogSensor(String deviceName) {
        super(deviceName );
    }


    @Override
    public double getSensorValue() {
        return getDevice().getMaxVoltage();
    }

}
