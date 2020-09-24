/*
 Copyright (c) 2019 HF Robotics (http://www.hfrobots.com)

 Permission is hereby granted, free of charge, to any person obtaining a copy
 of this software and associated documentation files (the "Software"), to deal
 in the Software without restriction, including without limitation the rights
 to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 copies of the Software, and to permit persons to whom the Software is
 furnished to do so, subject to the following conditions:
 The above copyright notice and this permission notice shall be included in all
 copies or substantial portions of the Software.
 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 SOFTWARE.
 */

package com.hfrobots.tnt.corelib.chaosninja;

import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import lombok.AccessLevel;
import lombok.Getter;

public class ChaoticMotor implements DcMotorEx {

    public enum MotorFailureMode {
        HEALTHY,
        DEAD,
        SLOW
    }

    private double actualPower = 0;

    private RunMode actualRunMode;

    private ZeroPowerBehavior actualZeroPowerBehavior;

    @Getter(AccessLevel.PACKAGE)
    private final DcMotorEx actualDcMotor;

    public ChaoticMotor(DcMotorEx actualDcMotor) {
        this.actualDcMotor = actualDcMotor;
        this.actualPower = actualDcMotor.getPower();
        this.actualRunMode = actualDcMotor.getMode();
        this.actualZeroPowerBehavior = actualDcMotor.getZeroPowerBehavior();
    }

    @Getter
    private MotorFailureMode motorFailureMode = MotorFailureMode.HEALTHY;

    public void setMotorFailureMode(MotorFailureMode motorFailureMode) {
        this.motorFailureMode = motorFailureMode;

        switch (motorFailureMode) {
            case HEALTHY:
                actualDcMotor.setZeroPowerBehavior(actualZeroPowerBehavior);
                actualDcMotor.setMode(actualRunMode);
                break;
            case DEAD:
                actualDcMotor.setZeroPowerBehavior(ZeroPowerBehavior.FLOAT);
                break;
        }
    }

    @Override
    public void setMotorEnable() {
        actualDcMotor.setMotorEnable();
    }

    @Override
    public void setMotorDisable() {
        actualDcMotor.setMotorDisable();
    }

    @Override
    public boolean isMotorEnabled() {
        return actualDcMotor.isMotorEnabled();
    }

    @Override
    public void setVelocity(double angularRate) {
        actualDcMotor.setVelocity(angularRate);
    }

    @Override
    public void setVelocity(double angularRate, AngleUnit unit) {
        actualDcMotor.setVelocity(angularRate, unit);
    }

    @Override
    public double getVelocity() {
        return actualDcMotor.getVelocity();
    }

    @Override
    public double getVelocity(AngleUnit unit) {
        return actualDcMotor.getVelocity(unit);
    }

    @Override
    @Deprecated
    public void setPIDCoefficients(RunMode mode, PIDCoefficients pidCoefficients) {
        actualDcMotor.setPIDCoefficients(mode, pidCoefficients);
    }

    @Override
    public void setPIDFCoefficients(RunMode mode, PIDFCoefficients pidfCoefficients) throws UnsupportedOperationException {
        actualDcMotor.setPIDFCoefficients(mode, pidfCoefficients);
    }

    @Override
    public void setVelocityPIDFCoefficients(double p, double i, double d, double f) {
        actualDcMotor.setVelocityPIDFCoefficients(p, i, d, f);
    }

    @Override
    public void setPositionPIDFCoefficients(double p) {
        actualDcMotor.setPositionPIDFCoefficients(p);
    }

    @Override
    @Deprecated
    public PIDCoefficients getPIDCoefficients(RunMode mode) {
        return actualDcMotor.getPIDCoefficients(mode);
    }

    @Override
    public PIDFCoefficients getPIDFCoefficients(RunMode mode) {
        return actualDcMotor.getPIDFCoefficients(mode);
    }

    @Override
    public void setTargetPositionTolerance(int tolerance) {
        actualDcMotor.setTargetPositionTolerance(tolerance);
    }

    @Override
    public int getTargetPositionTolerance() {
        return actualDcMotor.getTargetPositionTolerance();
    }

    @Override
    public double getCurrent(CurrentUnit unit) {
        return actualDcMotor.getCurrent(unit);
    }

    @Override
    public double getCurrentAlert(CurrentUnit unit) {
        return actualDcMotor.getCurrentAlert(unit);
    }

    @Override
    public void setCurrentAlert(double current, CurrentUnit unit) {
        actualDcMotor.setCurrentAlert(current, unit);
    }

    @Override
    public boolean isOverCurrent() {
        return actualDcMotor.isOverCurrent();
    }

    @Override
    public MotorConfigurationType getMotorType() {
        return actualDcMotor.getMotorType();
    }

    @Override
    public void setMotorType(MotorConfigurationType motorType) {
        actualDcMotor.setMotorType(motorType);
    }

    @Override
    public DcMotorController getController() {
        return actualDcMotor.getController();
    }

    @Override
    public int getPortNumber() {
        return actualDcMotor.getPortNumber();
    }

    @Override
    public void setZeroPowerBehavior(ZeroPowerBehavior zeroPowerBehavior) {
        actualZeroPowerBehavior = zeroPowerBehavior;

        if (motorFailureMode == MotorFailureMode.DEAD) {
            zeroPowerBehavior = ZeroPowerBehavior.FLOAT;
        }

        actualDcMotor.setZeroPowerBehavior(zeroPowerBehavior);
    }

    @Override
    public ZeroPowerBehavior getZeroPowerBehavior() {
        return actualZeroPowerBehavior;
    }

    @Override
    @Deprecated
    public void setPowerFloat() {
        actualDcMotor.setPowerFloat();
    }

    @Override
    public boolean getPowerFloat() {
        return actualDcMotor.getPowerFloat();
    }

    @Override
    public void setTargetPosition(int position) {
        actualDcMotor.setTargetPosition(position);
    }

    @Override
    public int getTargetPosition() {
        return actualDcMotor.getTargetPosition();
    }

    @Override
    public boolean isBusy() {
        return actualDcMotor.isBusy();
    }

    @Override
    public int getCurrentPosition() {
        return actualDcMotor.getCurrentPosition();
    }

    @Override
    public void setMode(RunMode mode) {
        actualRunMode = mode;

        if (motorFailureMode == MotorFailureMode.DEAD) {
            mode = RunMode.RUN_WITHOUT_ENCODER;
        }
        actualDcMotor.setMode(mode);
    }

    @Override
    public RunMode getMode() {
        return actualRunMode;
    }

    @Override
    public void setDirection(Direction direction) {
        actualDcMotor.setDirection(direction);
    }

    @Override
    public Direction getDirection() {
        return actualDcMotor.getDirection();
    }

    @Override
    public void setPower(double power) {
        actualPower = power;

        switch (motorFailureMode) {
            case DEAD:
                power = 0;
                break;
            case SLOW:
                power /= 2;
                break;
        }

        actualDcMotor.setPower(power);
    }

    @Override
    public double getPower() {
        return actualPower; // we might be lying!
    }

    @Override
    public Manufacturer getManufacturer() {
        return actualDcMotor.getManufacturer();
    }

    @Override
    public String getDeviceName() {
        return actualDcMotor.getDeviceName();
    }

    @Override
    public String getConnectionInfo() {
        return actualDcMotor.getConnectionInfo();
    }

    @Override
    public int getVersion() {
        return actualDcMotor.getVersion();
    }

    @Override
    public void resetDeviceConfigurationForOpMode() {
        actualDcMotor.resetDeviceConfigurationForOpMode();
    }

    @Override
    public void close() {
        actualDcMotor.close();
    }
}
