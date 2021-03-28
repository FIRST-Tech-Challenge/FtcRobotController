package com.technototes.library.hardware;

import com.technototes.logger.Stated;

import java.util.function.DoubleSupplier;

/** Class for hardware devices with a sensor value
 * @author Alex Stedman
 */
public interface Sensored extends Stated<Double>, DoubleSupplier {
    /** Get the sensor value
     *
     * @return The value
     */
    double getSensorValue();

    @Override
    default Double getState(){
        return getSensorValue();
    }

    @Override
    default double getAsDouble(){
        return getSensorValue();
    }
}
