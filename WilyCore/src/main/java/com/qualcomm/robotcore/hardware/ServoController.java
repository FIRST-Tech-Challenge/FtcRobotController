/* Copyright (c) 2014, 2015 Qualcomm Technologies Inc

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Qualcomm Technologies Inc nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

package com.qualcomm.robotcore.hardware;

/**
 * Interface for working with Servo Controllers
 * <p>
 * Different servo controllers will implement this interface.
 */
@SuppressWarnings("unused")
public interface ServoController extends HardwareDevice {

    /**
     * PWM Status - is pwm enabled?
     */
    enum PwmStatus { ENABLED, DISABLED, MIXED }

    /**
     * Enables all of the servos connected to this controller
     */
    void pwmEnable();

    /**
     * Disables all of the servos connected to this controller
     */
    void pwmDisable();

    /**
     * Returns the enablement status of the collective set of servos connected to this controller
     * @return the enablement status of the collective set of servos connected to this controller
     */
    PwmStatus getPwmStatus();

    /**
     * Set the position of a servo at the given channel
     * @param servo channel of servo
     * @param position from 0.0 to 1.0
     */
    void setServoPosition(int servo, double position);

    /**
     * Get the position of a servo at a given channel
     * @param servo channel of servo
     * @return position, scaled from 0.0 to 1.0
     */
    double getServoPosition(int servo);
}