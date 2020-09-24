/**
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
 **/

package com.hfrobots.tnt.fakes.drive;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.util.Range;

public class FakeServo implements Servo {
    protected Direction       direction        = Direction.FORWARD;
    protected double          limitPositionMin = MIN_POSITION;
    protected double          limitPositionMax = MAX_POSITION;

    private double servoPosition;
    @Override
    public ServoController getController() {
        throw new IllegalArgumentException("Not implemented");
    }

    @Override
    public int getPortNumber() {
        return 0;
    }

    @Override
    public void setDirection(Direction direction) {
        this.direction = direction;
    }

    @Override
    public Direction getDirection() {
        return direction;
    }

    @Override
    public void setPosition(double position) {
        position = Range.clip(position, MIN_POSITION, MAX_POSITION);

        double scaled = Range.scale(position, MIN_POSITION, MAX_POSITION, limitPositionMin, limitPositionMax);

        servoPosition = scaled;
    }

    @Override
    public double getPosition() {
        double reportedPosition = servoPosition;

        double scaled = Range.scale(reportedPosition, limitPositionMin, limitPositionMax, MIN_POSITION, MAX_POSITION);

        return Range.clip(scaled, MIN_POSITION, MAX_POSITION);
    }

    @Override
    public void scaleRange(double min, double max) {
        min = Range.clip(min, MIN_POSITION, MAX_POSITION);
        max = Range.clip(max, MIN_POSITION, MAX_POSITION);

        if (min >= max) {
            throw new IllegalArgumentException("min must be less than max");
        }

        limitPositionMin = min;
        limitPositionMax = max;
    }

    @Override
    public Manufacturer getManufacturer() {
        throw new IllegalArgumentException("Not implemented");
    }

    @Override
    public String getDeviceName() {
        throw new IllegalArgumentException("Not implemented");
    }

    @Override
    public String getConnectionInfo() {
        throw new IllegalArgumentException("Not implemented");
    }

    @Override
    public int getVersion() {
        return 0;
    }

    @Override
    public void resetDeviceConfigurationForOpMode() {
        this.limitPositionMin = MIN_POSITION;
        this.limitPositionMax = MAX_POSITION;
        this.direction = Direction.FORWARD;
    }

    @Override
    public void close() {

    }

    private double reverse(double position) {
        return MAX_POSITION - position + MIN_POSITION;
    }
}
