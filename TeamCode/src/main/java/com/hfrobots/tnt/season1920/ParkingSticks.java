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


public class ParkingSticks {
    private Servo stickServo0;

    private Servo stickServo1;

    private final double STICK_SERVO_0_STOWED_POSITION = 0;

    private final double STICK_SERVO_0_DEPLOYED_POSITION = .72;

    private final double STICK_SERVO_1_STOWED_POSITION = 0;

    private final double STICK_SERVO_1_DEPLOYED_POSITION = .74;

    public ParkingSticks(SimplerHardwareMap hardwareMap) {
        stickServo0 = hardwareMap.get(Servo.class, "parkingStick0");
        ServoUtil.setupPwmForRevSmartServo(stickServo0);
        stickServo0.setPosition(STICK_SERVO_0_STOWED_POSITION);

        stickServo1 = hardwareMap.get(Servo.class, "parkingStick1");
        ServoUtil.setupPwmForRevSmartServo(stickServo1);
        stickServo1.setPosition(STICK_SERVO_1_STOWED_POSITION);
    }

    public void stow() {
        stickServo0.setPosition(STICK_SERVO_0_STOWED_POSITION);
        stickServo1.setPosition(STICK_SERVO_1_STOWED_POSITION);

    }

    public void deploy() {
        stickServo0.setPosition(STICK_SERVO_0_DEPLOYED_POSITION);
        stickServo1.setPosition(STICK_SERVO_1_DEPLOYED_POSITION);
    }
}
