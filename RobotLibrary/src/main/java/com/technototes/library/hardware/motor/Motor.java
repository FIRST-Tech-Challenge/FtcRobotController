package com.technototes.library.hardware.motor;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;
import com.technototes.library.hardware.Followable;
import com.technototes.library.hardware.HardwareDevice;
import com.technototes.library.hardware.Invertable;
import com.technototes.logger.Log;
import com.technototes.library.util.UnsupportedFeatureException;

public class Motor<T extends DcMotorSimple> extends HardwareDevice<T> implements Invertable<Motor>, Followable<Motor> {
    public Motor(T d) {
        super(d);
    }

    public Motor(HardwareDevice<T> m) {
        super(m.getDevice());
    }

    public Motor(String s) {
        super(s);
    }


    @Override
    public boolean getInverted() {
        return device.getDirection() == DcMotorSimple.Direction.FORWARD;
    }

    @Override
    public Motor setInverted(boolean val) {
        device.setDirection(val ? DcMotorSimple.Direction.FORWARD : DcMotorSimple.Direction.REVERSE);
        return this;
    }

    public void setSpeedWithScale(double val, double scale) {
        device.setPower(val * scale);
    }

    @Log
    public double getSpeed() {
        return device.getPower();
    }

    public void setSpeed(double val) {
        device.setPower(Range.clip(val, -1, 1));
    }

    @Override
    public Motor follow(Motor d) {
        return new MotorGroup(d, this);
    }

    public void setIdleBehavior(DcMotor.ZeroPowerBehavior b) throws UnsupportedFeatureException {
        if (device instanceof DcMotor) {
            ((DcMotor) device).setZeroPowerBehavior(b);
        } else {
            throw new UnsupportedFeatureException("Idle behavior for CRServos", "in the SDK it does not exist");
        }
    }

}
