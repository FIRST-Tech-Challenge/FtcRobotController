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

/**
 * For hardware devices which are manipulated using a pulse width modulation (PWM) signal,
 * the {@link PwmControl} interface provides control of the width of pulses used and whether
 * the PWM is enabled or disabled.
 *
 * <p>PWM is commonly used to control servos. {@link PwmControl} is thus found as a second
 * interface on servo objects whose primary interface is {@link Servo} or {@link CRServo}
 * when the underlying servo controller hardware supports this fine-grained control (not all
 * servo controllers provide such control). To access the {@link PwmControl} interface,
 * cast your {@link Servo} or {@link CRServo} object to {@link PwmControl}; however, it is
 * usually prudent to first test whether the cast will succeed by testing using <pre>instanceof</pre>.
 *
 * @see DcMotorEx
 * @see <a href="https://en.wikipedia.org/wiki/Pulse-width_modulation">Pulse-width modulation</a>
 */
public interface PwmControl
{
    /**
     * Sets the PWM range limits for the servo
     * @param range the new PWM range limits for the servo
     * @see #getPwmRange()
     */
    void setPwmRange(PwmRange range);

    /**
     * Returns the current PWM range limits for the servo
     * @return the current PWM range limits for the servo
     * @see #setPwmRange(PwmRange)
     */
    PwmRange getPwmRange();

    /**
     * Individually energizes the PWM for this particular servo.
     * @see #setPwmDisable()
     * @see #isPwmEnabled()
     */
    void setPwmEnable();

    /**
     * Individually denergizes the PWM for this particular servo
     * @see #setPwmEnable()
     */
    void setPwmDisable();

    /**
     * Returns whether the PWM is energized for this particular servo
     * @see #setPwmEnable()
     */
    boolean isPwmEnabled();

    /**
     * PwmRange instances are used to specify the upper and lower pulse widths
     * and overall framing rate for a servo.
     *
     * @see <a href="http://www.endurance-rc.com/ppmtut.php">Guide to PWM and PPM</a>
     * @see <a href="https://www.servocity.com/html/hs-485hb_servo.html">HS-485HB servo information</a>
     */
    class PwmRange
    {

        /** usFrameDefault is the default frame rate used, in microseconds */
        public static final double usFrameDefault = 20000;
        public static final double usPulseUpperDefault = 2400;
        public static final double usPulseLowerDefault = 600;

        /** defaultRange is the default PWM range used */
        public final static PwmRange defaultRange = new PwmRange(usPulseLowerDefault, usPulseUpperDefault);

        /** usPulseLower is the minimum PWM rate used, in microseconds. This corresponds to a servo position of 0.0. */
        public final double usPulseLower;
        /** usPulseLower is the maximum PWM rate used, in microseconds. This corresponds to a servo position of 1.0. */
        public final double usPulseUpper;
        /** usFrame is the rate, in microseconds, at which the PWM is transmitted. */
        public final double usFrame;

        /**
         * Creates a new PwmRange with the indicated lower and upper bounds and the default
         * framing rate.
         * @param usPulseLower the minimum PWM rate used, in microsecond
         * @param usPulseUpper the maximum PWM rate used, in microseconds
         */
        public PwmRange(double usPulseLower, double usPulseUpper)
        {
            this(usPulseLower, usPulseUpper, usFrameDefault);
        }

        /**
         * Creates a new PwmRange with the indicated lower and upper bounds and the specified
         * framing rate.
         * @param usPulseLower the minimum PWM rate used, in microsecond
         * @param usPulseUpper the maximum PWM rate used, in microseconds
         * @param usFrame the framing rate, in microseconds
         */
        public PwmRange(double usPulseLower, double usPulseUpper, double usFrame)
        {
            this.usPulseLower = usPulseLower;
            this.usPulseUpper = usPulseUpper;
            this.usFrame = usFrame;
        }

        @Override
        public boolean equals(Object o)
        {
            if (o instanceof PwmRange)
            {
                PwmRange him = (PwmRange)o;
                return this.usPulseLower==him.usPulseLower
                        && this.usPulseUpper==him.usPulseUpper
                        && this.usFrame==him.usFrame;
            }
            return false;
        }

        @Override
        public int hashCode()
        {
            return ((Double)this.usPulseLower).hashCode()
                    ^ ((Double)this.usPulseUpper).hashCode()
                    ^ ((Double)this.usFrame).hashCode();
        }
    }
}
