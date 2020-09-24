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

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;

import lombok.AccessLevel;
import lombok.Getter;

public class ChaoticServo implements Servo {

    public double actualPosition;

    public enum ServoFailureMode {
        DEAD,
        REVERSED,
        HEALTHY
    }

    private ServoFailureMode failureMode = ServoFailureMode.HEALTHY;

    @Getter(AccessLevel.PACKAGE)
    private final Servo actualServo;

    public ChaoticServo(final Servo actualServo) {
        this.actualServo = actualServo;
        this.actualPosition = actualServo.getPosition();
    }

    protected void setFailureMode(ServoFailureMode failureMode) {
        this.failureMode = failureMode;
    }

    @Override
    public ServoController getController() {
        return actualServo.getController();
    }

    @Override
    public int getPortNumber() {
        return actualServo.getPortNumber();
    }

    @Override
    public void setDirection(Direction direction) {
        actualServo.setDirection(direction);
    }

    @Override
    public Direction getDirection() {
        return actualServo.getDirection();
    }

    @Override
    public void setPosition(double position) {
        actualPosition = position;

        if (failureMode == ServoFailureMode.DEAD) {
            return;
        }

        actualServo.setPosition(position);
    }

    @Override
    public double getPosition() {
        return actualServo.getPosition();
    }

    @Override
    public void scaleRange(double min, double max) {
        actualServo.scaleRange(min, max);
    }

    @Override
    public Manufacturer getManufacturer() {
        return actualServo.getManufacturer();
    }

    @Override
    public String getDeviceName() {
        return actualServo.getDeviceName();
    }

    @Override
    public String getConnectionInfo() {
        return actualServo.getConnectionInfo();
    }

    @Override
    public int getVersion() {
        return actualServo.getVersion();
    }

    @Override
    public void resetDeviceConfigurationForOpMode() {
        actualServo.resetDeviceConfigurationForOpMode();
    }

    @Override
    public void close() {
        actualServo.close();
    }
}
