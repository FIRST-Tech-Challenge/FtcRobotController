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

import com.ftc9929.testing.fakes.FakeTelemetry;
import com.ftc9929.testing.fakes.drive.FakeDcMotorEx;
import com.google.common.base.Ticker;
import com.google.common.testing.FakeTicker;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.junit.Assert;
import org.junit.Before;
import org.junit.Test;

import java.util.concurrent.TimeUnit;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

public class LauncherTest {
    private HardwareMap hwMap;

    private FakeDcMotorEx frontLauncherMotor;

    private FakeDcMotorEx rearLauncherMotor;

    private FakeTicker ticker = new FakeTicker();

    private Launcher launcher;

    @Before
    public void setup() {
        FakeTelemetry fakeTelemetry = new FakeTelemetry();
        fakeTelemetry.setOutputToStdout(true);

        launcher = new Launcher(UltimateGoalTestConstants.HARDWARE_MAP, fakeTelemetry, ticker);
        frontLauncherMotor = (FakeDcMotorEx)UltimateGoalTestConstants.HARDWARE_MAP.get(DcMotorEx.class, "frontLauncherMotor");
        rearLauncherMotor = (FakeDcMotorEx)UltimateGoalTestConstants.HARDWARE_MAP.get(DcMotorEx.class, "rearLauncherMotor");
    }

    @Test
    public void velocityTracking() {
        assertFalse(launcher.isLauncherAtFullSpeed());

        frontLauncherMotor.setCurrentPosition(1000);
        rearLauncherMotor.setCurrentPosition(1000);
        assertFalse(launcher.isLauncherAtFullSpeed());

        ticker.advance(120, TimeUnit.MILLISECONDS);

        frontLauncherMotor.setCurrentPosition(2000);
        rearLauncherMotor.setCurrentPosition(2000);
        assertFalse(launcher.isLauncherAtFullSpeed());

        frontLauncherMotor.setCurrentPosition(2000 + (int)((double)Launcher.LAUNCH_SPEED_ENC_SEC / 1000D * 120D));
        rearLauncherMotor.setCurrentPosition(2000 + (int)((double)Launcher.LAUNCH_SPEED_ENC_SEC / 1000D * 120D));

        ticker.advance(120, TimeUnit.MILLISECONDS);

        assertTrue(launcher.isLauncherAtFullSpeed());
    }

}
