package com.technototes.library.hardware.motor;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.technototes.library.hardware.HardwareDevice;
import com.technototes.library.hardware.PID;
import com.technototes.library.hardware.Sensored;
import com.technototes.library.hardware.sensor.encoder.Encoder;
import com.technototes.library.hardware.sensor.encoder.MotorEncoderSensor;
import com.technototes.library.measurement.unit.MotorEncoderUnit;
import com.technototes.logger.Log;
import com.technototes.library.util.MathUtils;
import com.technototes.library.util.PIDUtils;

public class EncodedMotor<T extends DcMotor> extends Motor<T> implements Sensored, PID {

    public PIDCoefficients coefficients;
    public double threshold = 50;
    public MotorEncoderSensor encoder;

    public EncodedMotor(T d, MotorEncoderUnit e) {
        super(d);
        coefficients = new PIDCoefficients(0, 0, 0);
        encoder = new MotorEncoderSensor(d, e);
    }

    public EncodedMotor(HardwareDevice<T> m, MotorEncoderUnit e) {
        super(m.getDevice());
        coefficients = new PIDCoefficients(0, 0, 0);
        encoder = new MotorEncoderSensor(device, e);
    }

    public EncodedMotor(String s, MotorEncoderUnit e) {
        super(s);
        coefficients = new PIDCoefficients(0, 0, 0);
        //controller = new PIDFController(coefficients);
        encoder = new MotorEncoderSensor(device, e);
    }

    public EncodedMotor setPID(double p, double i, double d){
        coefficients = new PIDCoefficients(p, i, d);
        return this;
    }

    @Override
    public EncodedMotor setInverted(boolean val) {
        device.setDirection(val ? DcMotorSimple.Direction.FORWARD : DcMotorSimple.Direction.REVERSE);
        return this;
    }

    @Override
    public double getSensorValue() {
        return encoder.getSensorValue();
    }

    public MotorEncoderUnit getSensorUnit(){
        return encoder.unit;
    }
    @Override
    public void setPIDValues(double p, double i, double d) {
        coefficients = new PIDCoefficients(p, i, d);
    }

    @Override
    public boolean setPositionPID(double val) {
        if (!isAtPosition(val)) {
            setSpeed(MathUtils.constrain(-0.1,(val-getSensorValue())/(coefficients.kP*10000), 0.1)*10);
        } else {
            setSpeed(0);
            return true;
        }
        return false;
    }

    public boolean setPosition(double ticks) {
        return setPosition(ticks, 0.5);
    }

    public boolean setPosition(double ticks, double speed) {
        if (!isAtPosition(ticks)) {
            setSpeed(getSensorValue() < ticks ? speed : -speed);
        } else {
            setSpeed(0);
            return true;
        }
        return false;
    }

    public boolean isAtPosition(double ticks) {
        return Math.abs(ticks - getSensorValue()) < threshold;
    }

    public void resetEncoder() {
        encoder.zeroEncoder();
    }

    @Override
    @Log
    public double getSpeed() {
        return super.getSpeed();
    }
}
