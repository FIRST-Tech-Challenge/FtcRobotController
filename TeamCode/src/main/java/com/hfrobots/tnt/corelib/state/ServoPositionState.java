/**
 Copyright (c) 2018 HF Robotics (http://www.hfrobots.com)
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

package com.hfrobots.tnt.corelib.state;

import com.hfrobots.tnt.corelib.control.DebouncedGamepadButtons;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * A simple State implementation that puts the given servo in the given position and
 * then transitions to the next State.
 */
public class ServoPositionState extends State {
    Servo servo;
    private double servoPosition;

    public ServoPositionState(final String name, final Telemetry telemetry, Servo servo, double servoPosition ) {
        super(name, telemetry);
        this.servo = servo;
        this.servoPosition = servoPosition;
    }

    @Override
    public State doStuffAndGetNextState() {
        if (servo != null) {
            servo.setPosition(servoPosition);
        }

        return nextState;
    }

    @Override
    public void resetToStart() {
        // TODO: If we wanted to do this, how could we reset the servo to the position
        // it was in before the state started?
    }

    @Override
    public void liveConfigure(DebouncedGamepadButtons buttons) {

    }
}
