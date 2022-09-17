/*
 * Copyright (c) 2019 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.openftc.easyopencv;

import org.opencv.core.Mat;

import java.util.ArrayList;

public class OpenCvTrackerApiPipeline extends OpenCvPipeline
{
    private ArrayList<OpenCvTracker> trackers = new ArrayList<>();
    private int trackerDisplayIdx = 0;

    public synchronized void addTracker(OpenCvTracker tracker)
    {
        trackers.add(tracker);
    }

    public synchronized void removeTracker(OpenCvTracker tracker)
    {
        trackers.remove(tracker);

        if(trackerDisplayIdx >= trackers.size())
        {
            trackerDisplayIdx--;

            if(trackerDisplayIdx < 0)
            {
                trackerDisplayIdx = 0;
            }
        }
    }

    @Override
    public synchronized Mat processFrame(Mat input)
    {
        if(trackers.size() == 0)
        {
            return input;
        }

        ArrayList<Mat> returnMats = new ArrayList<>(trackers.size());

        for(OpenCvTracker tracker : trackers)
        {
            returnMats.add(tracker.processFrameInternal(input));
        }

        return returnMats.get(trackerDisplayIdx);
    }

    @Override
    public synchronized void onViewportTapped()
    {
        trackerDisplayIdx++;

        if(trackerDisplayIdx >= trackers.size())
        {
            trackerDisplayIdx = 0;
        }
    }
}
