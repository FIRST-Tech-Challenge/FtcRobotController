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

package com.hfrobots.tnt.corelib.metrics;

/**
 * An adapter for GaugeMetricSources that always sends the requested "interesting"
 * values, otherwise samples
 */

public class InterestingValueMetricSource implements GaugeMetricSource {
    private final GaugeMetricSource realSource;

    private final int numToReportAtEdges;

    private final double epsilon;

    private final SamplingMetricSource samplingMetricSource;

    private double previousReportedValue;

    private boolean haveSampledOnce = false;

    private int edgeReportingCountdown;

    public InterestingValueMetricSource(GaugeMetricSource realSource, int numToReportAtEdges, int sampleFrequency, double epsilon) {
        this.realSource = realSource;
        this.numToReportAtEdges = numToReportAtEdges;
        this.epsilon = epsilon;
        this.samplingMetricSource = new SamplingMetricSource(realSource, sampleFrequency);
    }


    @Override
    public String getSampleName() {
        return realSource.getSampleName();
    }

    @Override
    public double getValue() {
        double sampledValue = realSource.getValue();

        if (!haveSampledOnce) {
            edgeReportingCountdown = numToReportAtEdges;
            haveSampledOnce = true;
        } else if (Math.abs(sampledValue - previousReportedValue) > epsilon) {
            previousReportedValue = sampledValue;
            edgeReportingCountdown = numToReportAtEdges;
        }

        if (edgeReportingCountdown > 0) {
            // report all values for awhile...
            edgeReportingCountdown--;

            return sampledValue;
        } else {
            return samplingMetricSource.getValue();
        }
    }
}
