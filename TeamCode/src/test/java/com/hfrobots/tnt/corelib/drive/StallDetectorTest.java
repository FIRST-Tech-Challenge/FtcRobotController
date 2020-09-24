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

import com.google.common.collect.ImmutableList;
import com.google.common.testing.FakeTicker;

import org.junit.Assert;
import org.junit.Test;

import java.util.List;
import java.util.concurrent.TimeUnit;

public class StallDetectorTest {

    static class TimeseriesValue {
        private final long timestampMillis;

        private final double value;

        TimeseriesValue(final long timestampMillis, final double value) {
            this.timestampMillis = timestampMillis;
            this.value = value;
        }
    }

    private final static List<TimeseriesValue> noStallValues;

    private final static List<TimeseriesValue> stallValues;

    static {
        // These values are from our SkyStone Robot v1
        // Please add more observations from other robots, and robot variations to make
        // our StalLDetector more robust!

        noStallValues = ImmutableList.of(
                new TimeseriesValue(860, 61),
                new TimeseriesValue(871, 61),
                new TimeseriesValue(879, 59),
                new TimeseriesValue(884, 58),
                new TimeseriesValue(904, 53),
                new TimeseriesValue(911, 51),
                new TimeseriesValue(919, 47),
                new TimeseriesValue(926, 45),
                new TimeseriesValue(933, 42),
                new TimeseriesValue(941, 49));
                
        stallValues = ImmutableList.of(
                new TimeseriesValue(526, 213.0),
                new TimeseriesValue(563, 140.0),
                new TimeseriesValue(586, 101.0),
                new TimeseriesValue(607, 74.0),
                new TimeseriesValue(632, 55.0),
                new TimeseriesValue(663, 49.0),
                new TimeseriesValue(681, 49.0),
                new TimeseriesValue(699, 49.0),
                new TimeseriesValue(722, 49.0),
                new TimeseriesValue(750, 49.0),
                new TimeseriesValue(775, 49.0),
                new TimeseriesValue(795, 49.0),
                new TimeseriesValue(813, 49.0),
                new TimeseriesValue(840, 49.0),
                new TimeseriesValue(860, 49.0),
                new TimeseriesValue(882, 49.0),
                new TimeseriesValue(890, 49.0),
                new TimeseriesValue(897, 49.0),
                new TimeseriesValue(904, 49.0),
                new TimeseriesValue(911, 49.0),
                new TimeseriesValue(918, 49.0),
                new TimeseriesValue(925, 49.0),
                new TimeseriesValue(930, 49.0),
                new TimeseriesValue(935, 49.0),
                new TimeseriesValue(946, 49.0),
                new TimeseriesValue(951, 49.0),
                new TimeseriesValue(958, 49.0),
                new TimeseriesValue(963, 49.0),
                new TimeseriesValue(968, 49.0),
                new TimeseriesValue(974, 49.0),
                new TimeseriesValue(980, 49.0),
                new TimeseriesValue(985, 49.0),
                new TimeseriesValue(990, 49.0),
                new TimeseriesValue(995, 49.0),
                new TimeseriesValue(1001, 49.0),
                new TimeseriesValue(1008, 49.0));
    }

    @Test
    public void testStallDetectorNoStall() {
        FakeTicker fakeTicker = new FakeTicker();

        // Start at the first timestamp
        long priorTimestampMillis = 0;

        StallDetector stallDetector = new StallDetector(fakeTicker, 1, 250);

        for (TimeseriesValue tsVal : noStallValues) {
            final long deltaMillis;

            if (priorTimestampMillis == 0) {
                deltaMillis = tsVal.timestampMillis;
            } else {
                deltaMillis = tsVal.timestampMillis - priorTimestampMillis;
            }

            priorTimestampMillis = tsVal.timestampMillis;

            fakeTicker.advance(deltaMillis, TimeUnit.MILLISECONDS);

            // Now the test - check that the stall detector never says it is stalled
            Assert.assertFalse(stallDetector.isStalled(tsVal.value));
        }
    }

    @Test
    public void testStallDetectorStalled() {
        FakeTicker fakeTicker = new FakeTicker();

        // Start at the first timestamp
        long priorTimestampMillis = 0;

        StallDetector stallDetector = new StallDetector(fakeTicker, 1, 250);

        boolean detectedStall = false;

        for (TimeseriesValue tsVal : stallValues) {
            final long deltaMillis;

            if (priorTimestampMillis == 0) {
                deltaMillis = tsVal.timestampMillis;
            } else {
                deltaMillis = tsVal.timestampMillis - priorTimestampMillis;
            }

            priorTimestampMillis = tsVal.timestampMillis;

            fakeTicker.advance(deltaMillis, TimeUnit.MILLISECONDS);

            if (stallDetector.isStalled(tsVal.value)) {
                detectedStall = true;

                break;
            }

        }

        // Now the test - check that the stall detector found the stall
        Assert.assertTrue(detectedStall);
    }
}
