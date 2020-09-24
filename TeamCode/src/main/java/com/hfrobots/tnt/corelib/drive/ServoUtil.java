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

package com.hfrobots.tnt.corelib.drive;

import android.util.Log;

import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.hardware.ServoControllerEx;

import lombok.NonNull;

import static com.hfrobots.tnt.corelib.Constants.LOG_TAG;

public class ServoUtil {

    /**
     * Sets the PWM range for a REV Smart Servo (500-2500us)
     * @param servo - the servo that requires a PWM range adjustment
     */
    public static void setupPwmForRevSmartServo(@NonNull final Servo servo) {
        int portNumber = servo.getPortNumber();
        try {
            ServoController servoController = servo.getController();

            if (servoController instanceof ServoControllerEx) {
                Log.d(LOG_TAG, "Setting REV PWM range for servo at port " + portNumber
                        + " on controller " + servoController.getConnectionInfo());

                PwmControl.PwmRange revPwmRange = new PwmControl.PwmRange(500, 2500);

                ((ServoControllerEx) servoController).setServoPwmRange(portNumber, revPwmRange);
            }
        } catch (IllegalArgumentException iae) {
            Log.e(LOG_TAG, "Servo does not have a controller");
        }
    }
}
