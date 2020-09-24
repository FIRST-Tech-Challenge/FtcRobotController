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

package com.hfrobots.tnt.corelib.state;

import com.google.common.base.Stopwatch;
import com.google.common.base.Ticker;
import com.hfrobots.tnt.corelib.control.DebouncedGamepadButtons;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.concurrent.TimeUnit;

import lombok.NonNull;

/**
 * A delay state that takes a stopwatch as the time source, to make it possible to advance the
 * elapsed time with a test ticker.
 */
public class StopwatchDelayState extends State {
    private long thresholdTimeMs;

    private final Stopwatch stopwatch;

    public StopwatchDelayState(@NonNull String name,
                               @NonNull Telemetry telemetry,
                               @NonNull Ticker ticker,
                               long val,
                               @NonNull TimeUnit unit) {
        super(name, telemetry);
        thresholdTimeMs = unit.toMillis(val);
        this.stopwatch = Stopwatch.createUnstarted(ticker);
    }

    @Override
    public void resetToStart() {
        stopwatch.reset();
    }

    @Override
    public void liveConfigure(DebouncedGamepadButtons buttons) {

    }

    @Override
    public State doStuffAndGetNextState() {
        if (!stopwatch.isRunning()) {
            stopwatch.start();

            return this;
        }

        long elapsedMs = stopwatch.elapsed(TimeUnit.MILLISECONDS);

        if (elapsedMs > thresholdTimeMs) {
            return nextState;
        }

        telemetry.addData("04", "Delay: %s %d of %d ms", name, elapsedMs, thresholdTimeMs);

        return this;
    }
}
