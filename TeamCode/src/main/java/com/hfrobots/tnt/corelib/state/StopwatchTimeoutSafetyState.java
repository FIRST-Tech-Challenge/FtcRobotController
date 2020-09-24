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

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.concurrent.TimeUnit;

import lombok.NonNull;

/**
 * Base class which can be used to build a State which has a safety timeout, uses a Ticker
 * and Stopwatch to allow testing of timeouts without waiting for elapsed wall clock time.
 *
 * The first call to isTimedOut() will start the timer, further calls will
 * return 'true' if the timeout specified in the constructor has expired
 */

public abstract class StopwatchTimeoutSafetyState extends State {
    protected final long safetyTimeoutMillis;
    private final Stopwatch stopwatch;

    protected StopwatchTimeoutSafetyState(@NonNull String name,
                                          @NonNull Telemetry telemetry,
                                          @NonNull Ticker ticker,
                                          long safetyTimeoutMillis) {
        super(name, telemetry);
        this.safetyTimeoutMillis = safetyTimeoutMillis;
        this.stopwatch = Stopwatch.createUnstarted(ticker);
    }

    protected boolean isTimedOut() {
        if (!stopwatch.isRunning()) {
            stopwatch.start();

            return false;
        }

        return stopwatch.elapsed(TimeUnit.MILLISECONDS) >= safetyTimeoutMillis;
    }

    @Override
    public void resetToStart() {
        resetTimer();
    }

    protected void resetTimer() {
        stopwatch.reset();
    }
}
