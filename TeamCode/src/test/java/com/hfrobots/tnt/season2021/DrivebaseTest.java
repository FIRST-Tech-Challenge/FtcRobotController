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

package com.hfrobots.tnt.season2021;

import com.ftc9929.testing.fakes.drive.FakeDcMotorEx;
import com.ftc9929.testing.fakes.util.FakeHardwareMapFactory;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.junit.Assert;
import org.junit.Before;
import org.junit.Test;

public class DrivebaseTest {
    HardwareMap hardwareMap;

    FakeDcMotorEx leftFront;
    FakeDcMotorEx leftRear;
    FakeDcMotorEx rightFront;
    FakeDcMotorEx rightRear;

    @Before
    public void setup() {
        hardwareMap = FakeHardwareMapFactory.getFakeHardwareMap("ugoal.xml");
        leftFront = (FakeDcMotorEx) hardwareMap.get(DcMotorEx.class, "leftFrontDriveMotor");
        leftRear = (FakeDcMotorEx) hardwareMap.get(DcMotorEx.class, "leftRearDriveMotor");
        rightFront = (FakeDcMotorEx) hardwareMap.get(DcMotorEx.class, "rightFrontDriveMotor");
        rightRear = (FakeDcMotorEx) hardwareMap.get(DcMotorEx.class, "rightRearDriveMotor");
    }

    @Test
    public void happyPath(){
        Drivebase drivebase = new Drivebase(hardwareMap);
        drivebase.driveCartesian(1, 0 ,0, false);

        // What does strafing look like?
        Assert.assertTrue(leftFront.getPower() == - leftRear.getPower());
        Assert.assertTrue(rightFront.getPower() == - rightRear.getPower());

        // What does turning counter-clockwise look like?

        // What does turning clockwise look like?

        // What does driving forward look like?
    }
}
