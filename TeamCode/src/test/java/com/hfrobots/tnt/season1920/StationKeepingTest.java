/**
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
 **/

package com.hfrobots.tnt.season1920;

import com.google.common.testing.FakeTicker;
import com.hfrobots.tnt.fakes.sensors.FakeDistanceSensor;
import com.hfrobots.tnt.fakes.FakeHardwareMap;
import com.hfrobots.tnt.fakes.FakeTelemetry;

import org.junit.Assert;
import org.junit.Before;
import org.junit.Test;

import java.util.concurrent.TimeUnit;

public class StationKeepingTest {
    private FakeHardwareMap hardwareMap;

    private FakeTicker ticker;

    private FakeDistanceSensor leftDistanceSensor;

    private FakeDistanceSensor rightDistanceSensor;

    private StationKeeping stationKeeping;

    @Before
    public void setUp() {
        ticker = new FakeTicker();
        ticker.setAutoIncrementStep(5, TimeUnit.SECONDS);

        hardwareMap = new FakeHardwareMap();

        leftDistanceSensor = new FakeDistanceSensor();
        rightDistanceSensor = new FakeDistanceSensor();

        hardwareMap.addDevice(StationKeeping.LEFT_DISTANCE_SENSOR_NAME, leftDistanceSensor);
        hardwareMap.addDevice(StationKeeping.RIGHT_DISTANCE_SENSOR_NAME, rightDistanceSensor);

        FakeTelemetry telemetry = new FakeTelemetry();
        telemetry.setOutputTelemetry(true);

        stationKeeping = new StationKeeping(hardwareMap, telemetry);
    }

    @Test
    public void needsRightTurn() {
        leftDistanceSensor.setDistance(StationKeeping.MINIMUM_DISTANCE_MM + 90);
        rightDistanceSensor.setDistance(StationKeeping.MINIMUM_DISTANCE_MM + 10);

        StationKeeping.StationKeepingSignals signals = stationKeeping.calculateSignals();

        Assert.assertTrue(signals.getTurnPower() > 0);
        Assert.assertEquals(StationKeeping.StationKeepingSignals.State.ACTIVE, signals.getState());

        leftDistanceSensor.setDistance(StationKeeping.MINIMUM_DISTANCE_MM + 50);
        rightDistanceSensor.setDistance(StationKeeping.MINIMUM_DISTANCE_MM + 50);

        signals = stationKeeping.calculateSignals();

        Assert.assertEquals(signals.getTurnPower(), 0, .01);

        // We need to extend our old PID code to use StopWatch, so we can deal with "settlingTime"
        // and make this assertion!
        // Assert.assertEquals(StationKeeping.StationKeepingSignals.State.ON_STATION, signals.getState());
    }

    @Test
    public void needsLeftTurn() {
        leftDistanceSensor.setDistance(StationKeeping.MINIMUM_DISTANCE_MM + 10);
        rightDistanceSensor.setDistance(StationKeeping.MINIMUM_DISTANCE_MM + 90);

        StationKeeping.StationKeepingSignals signals = stationKeeping.calculateSignals();

        Assert.assertTrue(signals.getTurnPower() < 0);
        Assert.assertEquals(StationKeeping.StationKeepingSignals.State.ACTIVE, signals.getState());

        leftDistanceSensor.setDistance(StationKeeping.MINIMUM_DISTANCE_MM + 50);
        rightDistanceSensor.setDistance(StationKeeping.MINIMUM_DISTANCE_MM + 50);

        signals = stationKeeping.calculateSignals();

        Assert.assertEquals(signals.getTurnPower(), 0, .01);
        // Assert.assertEquals(StationKeeping.StationKeepingSignals.State.ON_STATION, signals.getState());

    }

    @Test
    public void tooFar() {
        leftDistanceSensor.setDistance(StationKeeping.MAXIMUM_DISTANCE_MM + 10);
        rightDistanceSensor.setDistance(StationKeeping.MAXIMUM_DISTANCE_MM + 10);

        StationKeeping.StationKeepingSignals signals = stationKeeping.calculateSignals();

        Assert.assertEquals(signals.getTurnPower(), 0, .01);
        Assert.assertEquals(StationKeeping.StationKeepingSignals.State.TOO_FAR, signals.getState());
    }

    @Test
    public void tooClose() {
        leftDistanceSensor.setDistance(StationKeeping.MINIMUM_DISTANCE_MM - 10);
        rightDistanceSensor.setDistance(StationKeeping.MINIMUM_DISTANCE_MM - 10);

        StationKeeping.StationKeepingSignals signals = stationKeeping.calculateSignals();

        Assert.assertEquals(signals.getTurnPower(), 0, .01);
        Assert.assertEquals(StationKeeping.StationKeepingSignals.State.TOO_CLOSE, signals.getState());

    }

    @Test
    public void tooMuchError() {
        leftDistanceSensor.setDistance(StationKeeping.MINIMUM_DISTANCE_MM + 10);
        rightDistanceSensor.setDistance(StationKeeping.MAXIMUM_DISTANCE_MM + 10);

        StationKeeping.StationKeepingSignals signals = stationKeeping.calculateSignals();

        Assert.assertEquals(signals.getTurnPower(), 0, .01);
        Assert.assertEquals(StationKeeping.StationKeepingSignals.State.TOO_FAR, signals.getState());
    }
}