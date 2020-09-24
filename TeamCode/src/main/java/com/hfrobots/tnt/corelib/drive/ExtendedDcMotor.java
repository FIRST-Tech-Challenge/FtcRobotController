/**
 Copyright (c) 2016 HF Robotics (http://www.hfrobots.com)
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

package com.hfrobots.tnt.corelib.drive;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.Rotation;

/**
 * Additions to the DcMotor class that allow us to more easily use it
 * with drive trains
 */
public interface ExtendedDcMotor extends DcMotor {

    /**
     * Which way does the motor rotate when plugged in red:red black:black
     */
    Rotation getMotorNativeDirection();

    /**
     * Returns the number of encoder ticks/counts per full
     * revolution of the motor shaft
     *
     * @return encoder counts per revolution
     */
    double getEncoderCountsPerRevolution();

    /**
     * Sets a target position to run to relative to
     * the current absolute target position
     *
     * @return the absolute target position for the given relative position
     */
    int setRelativeTargetPosition(int position);

    /**
     * Returns the motor's current target position relative to the position
     * the motor had when setRelativeTargetPosition() was last called.
     *
     * @return relative target position
     */
    int getCurrentRelativePosition();

    /**
     * Resets the logical encoder count ... using STOP_AND_RESET_ENCODERS is an expensive
     * operation (it requires cycles).
     */
    void resetLogicalEncoderCount();

    /**
     * Returns the motor's current velocity relative to the last time the robot's velocity was recorded
     * @return relative velocity
     */
    //int getVelocity();

}
