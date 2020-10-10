package com.technototes.library.hardware.servo;

import com.technototes.library.hardware.*;
import com.technototes.library.util.PIDUtils;
import com.technototes.logger.Log;

public class Servo extends HardwareDevice<com.qualcomm.robotcore.hardware.Servo> implements Sensored, Invertable<Servo>, Followable<Servo>, PID {

    public double pid_p, pid_i, pid_d;

    public Servo(HardwareDevice<com.qualcomm.robotcore.hardware.Servo> d) {
        super(d);
    }

    public Servo(com.qualcomm.robotcore.hardware.Servo d) {
        super(d);
    }

    public Servo(String s) {
        super(s);
    }


    public Servo setStartingPosition(double pos) {
        device.setPosition(pos);
        return this;
    }

    @Override
    public boolean getInverted() {
        return device.getDirection() == com.qualcomm.robotcore.hardware.Servo.Direction.FORWARD;
    }

    @Override
    public Servo setInverted(boolean val) {
        device.setDirection(val ? com.qualcomm.robotcore.hardware.Servo.Direction.FORWARD : com.qualcomm.robotcore.hardware.Servo.Direction.REVERSE);
        return this;
    }

    public void setPosition(double val) {
        device.setPosition(val);
    }

    @Log
    @Override
    public double getSensorValue() {
        return device.getPosition();
    }

    public Servo setRange(double min, double max) {
        device.scaleRange(min, max);
        return this;
    }

    @Override
    public Servo follow(Servo d) {
        return new ServoGroup(this, d);
    }

    @Override
    public void setPIDValues(double p, double i, double d) {
        pid_p = p;
        pid_i = i;
        pid_d = d;
    }

    @Override
    public boolean setPositionPID(double val) {
        device.setPosition(PIDUtils.calculatePIDDouble(pid_p, pid_i, pid_d, device.getPosition(), val));
        return isAtPosition(val);
    }

    public boolean isAtPosition(double ticks) {
        return ticks - getSensorValue() > 0;
    }

}
