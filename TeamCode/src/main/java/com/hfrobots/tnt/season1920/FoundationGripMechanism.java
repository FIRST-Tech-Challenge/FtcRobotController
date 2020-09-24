/*
 Copyright (c) 2020 HF Robotics (http://www.hfrobots.com)
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

import com.hfrobots.tnt.corelib.util.SimplerHardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class FoundationGripMechanism {

    Servo leftFoundationGripServo;
    Servo rightFoundationGripServo;

    public final static double RIGHT_GRIP_SERVO_INIT = .823;
    public final static double RIGHT_GRIP_SERVO_DOWN = .177;
    public final static double RIGHT_GRIP_SERVO_UP = .68;

    public final static double LEFT_GRIP_SERVO_INIT = .136;
    public final static double LEFT_GRIP_SERVO_DOWN = .739;
    public final static double LEFT_GRIP_SERVO_UP = .262;

    public FoundationGripMechanism(SimplerHardwareMap hardwareMap) {

        leftFoundationGripServo = hardwareMap.get(Servo.class, "leftGripServo");

        rightFoundationGripServo = hardwareMap.get(Servo.class, "rightGripServo");

        // Move everything to initial physical state

        if (true) {
            initPos();
        }
    }

    public void initPos() {
        leftFoundationGripServo.setPosition(LEFT_GRIP_SERVO_INIT);
        rightFoundationGripServo.setPosition(RIGHT_GRIP_SERVO_INIT);
    }

    public void down() {
        leftFoundationGripServo.setPosition(LEFT_GRIP_SERVO_DOWN);
        rightFoundationGripServo.setPosition(RIGHT_GRIP_SERVO_DOWN);
    }

    public void up() {
        leftFoundationGripServo.setPosition(LEFT_GRIP_SERVO_UP);
        rightFoundationGripServo.setPosition(RIGHT_GRIP_SERVO_UP);
    }

}
