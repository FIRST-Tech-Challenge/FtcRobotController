/**
 Copyright (c) 2016 HF Robotics (http://www.hfrobots.com)
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

package com.hfrobots.tnt.corelib.state;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Base class which can be used to build a State which has a safety timeout
 *
 * The first call to isTimedOut() will start the timer, further calls will
 * return 'true' if the timeout specified in the constructor has expired
 */

public abstract class TimeoutSafetyState extends State {
    protected final long safetyTimeoutMillis;
    private long timeoutStartMillis;

    protected TimeoutSafetyState(String name, Telemetry telemetry, long safetyTimeoutMillis) {
        super(name, telemetry);
        this.safetyTimeoutMillis = safetyTimeoutMillis;
        timeoutStartMillis = 0;
    }

    protected boolean isTimedOut() {
        if (timeoutStartMillis == 0) {
            timeoutStartMillis = System.currentTimeMillis();

            return false;
        }


        return System.currentTimeMillis() - timeoutStartMillis >= safetyTimeoutMillis;
    }

    @Override
    public void resetToStart() {
        resetTimer();
    }

    protected void resetTimer() {
        timeoutStartMillis = 0;
    }
}
