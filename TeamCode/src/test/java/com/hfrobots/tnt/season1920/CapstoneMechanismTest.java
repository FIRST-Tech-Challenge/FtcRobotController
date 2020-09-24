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

import android.graphics.Paint;

import com.google.common.testing.FakeTicker;
import com.hfrobots.tnt.corelib.control.DebouncedButton;
import com.hfrobots.tnt.fakes.FakeHardwareMap;
import com.hfrobots.tnt.fakes.FakeTelemetry;
import com.hfrobots.tnt.fakes.control.FakeOnOffButton;
import com.hfrobots.tnt.fakes.drive.FakeServo;

import org.junit.Assert;
import org.junit.Before;
import org.junit.Test;

import java.util.concurrent.TimeUnit;

public class CapstoneMechanismTest {
    private FakeHardwareMap hardwareMap;

    private FakeServo fingerServo;

    private FakeServo capstoneServo;

    private FakeOnOffButton unsafeButton;

    private FakeOnOffButton dropButton;

    private FakeTicker ticker;

    private CapstoneMechanism capstoneMechanism;

    @Before
    public void setUp() {
        ticker = new FakeTicker();
        ticker.setAutoIncrementStep(5, TimeUnit.SECONDS);

        hardwareMap = new FakeHardwareMap();

        fingerServo = new FakeServo();
        capstoneServo = new FakeServo();

        dropButton = new FakeOnOffButton();
        unsafeButton = new FakeOnOffButton();

        hardwareMap.addDevice("fingerServo", fingerServo);
        hardwareMap.addDevice("capstoneServo", capstoneServo);

        FakeTelemetry telemetry = new FakeTelemetry();

        capstoneMechanism = new CapstoneMechanism(hardwareMap, telemetry,
                ticker);
        capstoneMechanism.setUnsafeButton(unsafeButton);
        capstoneMechanism.setDropButton(new DebouncedButton(dropButton));
    }

    @Test
    public void testSafeties() {
        fingerServo.setPosition(DeliveryMechanism.FINGER_UNGRIP);

        Assert.assertEquals(CapstoneMechanism.HOLDING_POSITION, capstoneServo.getPosition(), 0.001);
        Assert.assertEquals(DeliveryMechanism.FINGER_UNGRIP, fingerServo.getPosition(), 0.001);

        capstoneMechanism.periodicTask();
        dropButton.setPressed(true);

        capstoneMechanism.periodicTask();

        Assert.assertEquals(CapstoneMechanism.HOLDING_POSITION, capstoneServo.getPosition(), .001);

        dropButton.setPressed(false);
        capstoneMechanism.periodicTask();

        fingerServo.setPosition(DeliveryMechanism.FINGER_GRIP);
        dropButton.setPressed(true);
        capstoneMechanism.periodicTask();

        Assert.assertEquals(CapstoneMechanism.HOLDING_POSITION, capstoneServo.getPosition(), .001);

        dropButton.setPressed(false);
        capstoneMechanism.periodicTask();

        // Move the elapsed time into end-game territory....
        ticker.advance(75, TimeUnit.SECONDS);
        dropButton.setPressed(true);
        capstoneMechanism.periodicTask();

        Assert.assertEquals(CapstoneMechanism.DROPPING_POSITION, capstoneServo.getPosition(), .001);

        dropButton.setPressed(false);
        capstoneMechanism.periodicTask();

        dropButton.setPressed(true);
        capstoneMechanism.periodicTask();

        Assert.assertEquals(CapstoneMechanism.HOLDING_POSITION, capstoneServo.getPosition(), .001);
    }

    @Test
    public void testUnsafeButton() {
        fingerServo.setPosition(DeliveryMechanism.FINGER_UNGRIP);

        Assert.assertEquals(CapstoneMechanism.HOLDING_POSITION, capstoneServo.getPosition(), 0.001);
        Assert.assertEquals(DeliveryMechanism.FINGER_UNGRIP, fingerServo.getPosition(), 0.001);

        capstoneMechanism.periodicTask();
        dropButton.setPressed(true);
        unsafeButton.setPressed(true);

        capstoneMechanism.periodicTask();

        Assert.assertEquals(CapstoneMechanism.DROPPING_POSITION, capstoneServo.getPosition(), .001);
    }
}