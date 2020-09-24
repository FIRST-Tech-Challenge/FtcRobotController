/**
 Copyright (c) 2018 HF Robotics (http://www.hfrobots.com)
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

package com.hfrobots.tnt.corelib.control;

import static com.google.common.base.Preconditions.checkArgument;

import lombok.Builder;
import lombok.NonNull;

/**
 * Applies a parametric curve to the given RangeInput
 */
public class ParametricScaledRangeInput implements RangeInput {
    private final RangeInput rawInput;

    private final float throttleDeadband;
    private final float throttleGain;
    private final float throttleExponent;

    @Builder
    public ParametricScaledRangeInput(@NonNull RangeInput rawInput,
                                      float throttleDeadband,
                                      float throttleGain,
                                      float throttleExponent) {
        checkArgument(throttleDeadband >= 0,
                "Throttle deadband must be a non-negative value");

        checkArgument(throttleGain >= 0,
                "Throttle gain must be a non-negative value");

        checkArgument(throttleExponent >= 0,
                "Throttle exponent must be a non-negative value");

        this.rawInput = rawInput;
        this.throttleDeadband = throttleDeadband;
        this.throttleGain = throttleGain;
        this.throttleExponent = throttleExponent;
    }

    public float getPosition() {
        float rawPosition = rawInput.getPosition();

        return scaleThrottleValue(rawPosition);
    }

    float scaleThrottleValue(float unscaledValue) {
        return (float)((-1 * throttleDeadband) + (1 - throttleDeadband)
                * (throttleGain * Math.pow((double)unscaledValue, (double)throttleExponent)
                + (1F - throttleGain) * unscaledValue));
    }

    @Override
    public float getMaxPosition() {
        return rawInput.getMaxPosition();
    }

    @Override
    public float getMinPosition() {
        return rawInput.getMinPosition();
    }
}
