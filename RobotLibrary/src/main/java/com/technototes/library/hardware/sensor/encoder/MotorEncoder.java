package com.technototes.library.hardware.sensor.encoder;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.technototes.library.hardware.sensor.Sensor;
import com.technototes.logger.Log;

import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;

/** Class for encoders of motors
 * @author Alex Stedman
 */
public class MotorEncoder extends Sensor<DcMotor> implements Encoder {

    private int zero = 0;
    private DoubleSupplier supplier;

    /** Create motor encoder
     *
     * @param motor The motor to get the sensor from
     */
    public MotorEncoder(DcMotor motor) {
        super(motor);
        supplier = () -> motor.getCurrentPosition();
        zeroEncoder();
    }

    @Override
    public void zeroEncoder() {
        zero = (int) getSensorValue();
    }

    @Override
    public double getSensorValue() {
        return supplier.getAsDouble()-zero;
    }
}
