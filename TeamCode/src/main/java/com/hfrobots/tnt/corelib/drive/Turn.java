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

import org.firstinspires.ftc.robotcore.external.navigation.Rotation;

/**
 * A human-understandable description of a turn the robot must make during autonomous
 * with methods that are usable for steering with the gyro. Note that this class
 * follows the right-hand rule, so counter-clockwise turns are a positive relative
 * heading change
 */
public class Turn {
    private final Rotation direction;
    private int degrees;
    private int asHeading;

    public Turn(Rotation direction, int degrees) {
        this.direction = direction;
        this.degrees = degrees;

        final int headingSign;
        switch (this.direction) {
            case CW:
                headingSign = -1;
                break;
            case CCW:
                headingSign = 1;
                break;
            default:
                throw new IllegalArgumentException("Illegal direction specified: " + direction);
        }

        asHeading = headingSign * degrees;
    }

    public int getHeading() {
        return asHeading;
    }

    public int getDegrees() {
        return degrees;
    }

    public Rotation getDirection() {
        return direction;
    }

    /**
     * Returns an inverted representation of this Turn (i.e. for the opposite alliance)
     */
    public Turn invert() {
        return new Turn(invert(this.direction), degrees);
    }

    public static Rotation invert(Rotation toInvert) {
        switch (toInvert) {
            case CW:
                return Rotation.CCW;
            case CCW:
                return Rotation.CW;
            default:
                throw new IllegalArgumentException("Illegal rotation specified: " + toInvert);
        }
    }
}
