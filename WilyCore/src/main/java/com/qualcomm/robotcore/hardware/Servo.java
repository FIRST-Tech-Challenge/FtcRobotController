/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package com.qualcomm.robotcore.hardware;

import com.qualcomm.robotcore.hardware.configuration.ServoFlavor;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.ServoType;

/**
 * Instances of Servo interface provide access to servo hardware devices.
 */
@ServoType(flavor = ServoFlavor.STANDARD)
@DeviceProperties(name = "@string/configTypeServo", xmlTag = "Servo", builtIn = true)
public interface Servo extends HardwareDevice
{
    /**
     * The minimum allowable position to which a servo can be moved
     * @see #setPosition(double)
     */
    double MIN_POSITION = 0.0;
    /**
     * The maximum allowable position to which a servo can be moved
     * @see #setPosition(double)
     */
    double MAX_POSITION = 1.0;

    /**
     * Servos can be configured to internally reverse the values
     * to which their positioning power is set. This makes it easy, e.g.,
     * to have cooperating servos on two sides of a robot arm:
     * one would be set at at forward, the other at reverse, and the
     * difference between the two in that respect could be thereafter ignored.
     *
     * <p>At the start of an OpMode, servos are guaranteed to be in the forward direction.</p>
     *
     * @see #setDirection(Direction)
     */
    enum Direction { FORWARD, REVERSE }

    /**
     * Returns the underlying servo controller on which this servo is situated.
     * @return the underlying servo controller on which this servo is situated.
     * @see #getPortNumber()
     */
    ServoController getController();

    /**
     * Returns the port number on the underlying servo controller on which this motor is situated.
     * @return the port number on the underlying servo controller on which this motor is situated.
     * @see #getController()
     */
    int getPortNumber();

    /**
     * Sets the logical direction in which this servo operates.
     * @param direction the direction to set for this servo
     *
     * @see #getDirection()
     * @see com.qualcomm.robotcore.hardware.Servo.Direction
     */
    void setDirection(Direction direction);

    /**
     * Returns the current logical direction in which this servo is set as operating.
     * @return the current logical direction in which this servo is set as operating.
     * @see #setDirection(Direction)
     */
    Direction getDirection();

    /**
     * Sets the current position of the servo, expressed as a fraction of its available
     * range. If PWM power is enabled for the servo, the servo will attempt to move to
     * the indicated position.
     *
     * @param position the position to which the servo should move, a value in the range [0.0, 1.0]
     * @see ServoController#pwmEnable()
     * @see #getPosition()
     */
    void setPosition(double position);

    /**
     * Returns the position to which the servo was last commanded to move. Note that this method
     * does NOT read a position from the servo through any electrical means, as no such electrical
     * mechanism is, generally, available.
     * @return the position to which the servo was last commanded to move, or Double.NaN
     *         if no such position is known
     * @see #setPosition(double)
     * @see Double#NaN
     * @see Double#isNaN()
     */
    double getPosition();

    /**
     * Scales the available movement range of the servo to be a subset of its maximum range. Subsequent
     * positioning calls will operate within that subset range. This is useful if your servo has
     * only a limited useful range of movement due to the physical hardware that it is manipulating
     * (as is often the case) but you don't want to have to manually scale and adjust the input
     * to {@link #setPosition(double) setPosition()} each time.
     *
     * <p>For example, if scaleRange(0.2, 0.8) is set; then servo positions will be
     * scaled to fit in that range:<br>
     * setPosition(0.0) scales to 0.2<br>
     * setPosition(1.0) scales to 0.8<br>
     * setPosition(0.5) scales to 0.5<br>
     * setPosition(0.25) scales to 0.35<br>
     * setPosition(0.75) scales to 0.65<br>
     * </p>
     *
     * <p>Note the parameters passed here are relative to the underlying full range of motion of
     * the servo, not its currently scaled range, if any. Thus, scaleRange(0.0, 1.0) will reset
     * the servo to its full range of movement.</p>
     *
     * @param min    the lower limit of the servo movement range, a value in the interval [0.0, 1.0]
     * @param max    the upper limit of the servo movement range, a value in the interval [0.0, 1.0]
     *
     * @see #setPosition(double)
     */
    void scaleRange(double min, double max);
}
