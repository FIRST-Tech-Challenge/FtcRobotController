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

package com.hfrobots.tnt.season1920;

import com.hfrobots.tnt.corelib.drive.ServoUtil;
import com.hfrobots.tnt.corelib.util.SimplerHardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class SkystoneGrabber {
    private final Servo skystoneServo;

    private final double SKYSTONE_SERVO_STOWED_POSITION = 1;

    private final double SKYSTONE_SERVO_GRAB_POSITION = 0;

    public SkystoneGrabber(SimplerHardwareMap hardwareMap) {

        skystoneServo = hardwareMap.get(Servo.class, "skystoneServo");
        ServoUtil.setupPwmForRevSmartServo(skystoneServo);
        skystoneServo.setPosition(SKYSTONE_SERVO_STOWED_POSITION);

    }

    public void stow() {
        skystoneServo.setPosition(SKYSTONE_SERVO_STOWED_POSITION);

    }

    public void grab() {
        skystoneServo.setPosition(SKYSTONE_SERVO_GRAB_POSITION);
    }
}
