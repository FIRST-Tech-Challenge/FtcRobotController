/*
Copyright (c) 2018 Noah Andrews

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Noah Andrews nor the names of his contributors may be used to
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

package com.qualcomm.robotcore.hardware.configuration.annotations;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.configuration.ServoFlavor;

import java.lang.annotation.Documented;
import java.lang.annotation.ElementType;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;

/**
 * {@link ServoType} is an annotation with which a class or interface can be decorated in
 * order to define a new kind of servo that can be configured in the robot configuration user
 * interface.
 * <p>
 * If the servo is not connected to a REV Hub, only the flavor parameter will be applied, as
 * other servo controllers do not support setting a custom PWM range.
 * <p>
 * If the flavor is STANDARD or CONTINUOUS, this annotation can be placed on any class or interface.
 * <p>
 * If the flavor is CUSTOM, this annotation MUST implement {@link HardwareDevice} and have a
 * constructor with at least one of the following constructor signatures:
 * <ol>
 * <li>ctor(ServoControllerEx controller, int port)  -- Only used when configured on a REV Hub</li>
 * <li>ctor(ServoController controller, int port) -- Only used when configured on a CDIM</li>
 * </ol>
 * <p>
 * Must be accompanied by {@link DeviceProperties} annotation
 *
 * @see HardwareMap#get(Class, String)
 * @see HardwareMap#get(String)
 */
@Documented
@Target(ElementType.TYPE)
@Retention(RetentionPolicy.RUNTIME)
public @interface ServoType {
    /**
     * Indicates whether the servo is a standard servo, a continuous rotation servo,
     * or if you're defining an entirely new type of servo device
     */
    @NonNull ServoFlavor flavor();

    /**
     * Specifies the minimum PWM rate used, in microseconds. This corresponds to a servo position of 0.0.
     */
    double usPulseLower() default PwmControl.PwmRange.usPulseLowerDefault;

    /**
     * Specifies the maximum PWM rate used, in microseconds. This corresponds to a servo position of 1.0.
     */
    double usPulseUpper() default PwmControl.PwmRange.usPulseUpperDefault;

    /**
     * Specifies the rate, in microseconds, at which the PWM is transmitted
     */
    double usPulseFrameRate() default PwmControl.PwmRange.usFrameDefault;
}