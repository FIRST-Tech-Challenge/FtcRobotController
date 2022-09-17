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

import android.os.Debug;

import org.opencv.core.Mat;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;

import java.io.File;
import java.util.concurrent.Semaphore;

public abstract class OpenCvPipeline
{
    private boolean isFirstFrame = true;
    private static final Semaphore saveSemaphore = new Semaphore(5);
    private static final String defaultSavePath = "/sdcard/EasyOpenCV";

    private long firstFrameTimestamp;
    protected boolean MEMLEAK_DETECTION_ENABLED = true;
    protected int MEMLEAK_THRESHOLD_MB = 100;
    protected int MEMLEAK_DETECTION_PIPELINE_SETTLE_DELAY_SECONDS = 2;
    private long nativeAllocFirstMonitoredFrame;
    private boolean settled = false;
    private long currentAlloc;
    private long firstMonitoredFrameTimestamp;
    private String leakMsg = "";
    private long previousAbsoluteDeltaAlloc = Integer.MIN_VALUE;
    private double gcLeakOffsetMb;
    private int gcRuns = 0;
    private String lastLeakMsg = "";
    private long lastLeakMsgUpdateTime;

    public OpenCvPipeline()
    {
        synchronized (saveSemaphore)
        {
            File saveDir = new File(defaultSavePath);

            if(!saveDir.exists())
            {
                saveDir.mkdir();
            }
        }
    }

    Mat processFrameInternal(Mat input)
    {
        if(isFirstFrame)
        {
            init(input);
            firstFrameTimestamp = System.currentTimeMillis();
            isFirstFrame = false;
        }

        Mat ret = processFrame(input);
        leakDetection();
        return ret;
    }

    private void leakDetection()
    {
        if(!MEMLEAK_DETECTION_ENABLED)
        {
            return;
        }

        currentAlloc = Debug.getNativeHeapAllocatedSize();

        if(!settled && (System.currentTimeMillis() - firstFrameTimestamp) > MEMLEAK_DETECTION_PIPELINE_SETTLE_DELAY_SECONDS*1000)
        {
            settled = true;
            nativeAllocFirstMonitoredFrame = Debug.getNativeHeapAllocatedSize();
            firstMonitoredFrameTimestamp = System.currentTimeMillis();
            return;
        }

        if(!settled)
        {
            return;
        }

        long absoluteDeltaAlloc = currentAlloc - nativeAllocFirstMonitoredFrame;

        if(previousAbsoluteDeltaAlloc == Integer.MIN_VALUE)
        {
            previousAbsoluteDeltaAlloc = absoluteDeltaAlloc;
        }

        double previousAbsoluteDeltaAllocMB = previousAbsoluteDeltaAlloc / (1024.0*1024.0);

        double absoluteDeltaAllocMB = absoluteDeltaAlloc / (1024.0*1024.0);

        if((absoluteDeltaAllocMB-previousAbsoluteDeltaAllocMB) < -20)
        {
            System.out.println("GC probably ran");
            gcLeakOffsetMb += previousAbsoluteDeltaAllocMB;
            gcRuns++;
        }

        previousAbsoluteDeltaAlloc = absoluteDeltaAlloc;

        double timeSinceStartedMonitoring = (System.currentTimeMillis() - firstMonitoredFrameTimestamp)/1000.0;

        double leakRate = (absoluteDeltaAllocMB+gcLeakOffsetMb) / timeSinceStartedMonitoring;

        if((absoluteDeltaAllocMB+gcLeakOffsetMb) > MEMLEAK_THRESHOLD_MB)
        {
            leakMsg = String.format("Pipeline likely leaking memory @ %dMB/sec; total of %dMB leaked; GC likely run %d times", (int)leakRate, (int)(absoluteDeltaAllocMB+gcLeakOffsetMb), gcRuns);
        }
        else
        {
            leakMsg = "";
        }
    }

    String getLeakMsg()
    {
        if(System.currentTimeMillis() - lastLeakMsgUpdateTime > 250)
        {
            lastLeakMsgUpdateTime = System.currentTimeMillis();
            lastLeakMsg = leakMsg;
        }
        return lastLeakMsg;
    }

    public abstract Mat processFrame(Mat input);
    public void onViewportTapped() {}

    public void init(Mat mat) {}

    public void saveMatToDiskFullPath(Mat mat, final String fullPath)
    {
        try
        {
            saveSemaphore.acquire();
        }
        catch (InterruptedException e)
        {
            e.printStackTrace();
            return;
        }

        final Mat clone = mat.clone();

        new Thread(new Runnable()
        {
            @Override
            public void run()
            {
                try
                {
                    Imgproc.cvtColor(clone, clone, Imgproc.COLOR_RGB2BGR);
                    Imgcodecs.imwrite(fullPath, clone);
                }
                catch (Exception e)
                {
                    e.printStackTrace();
                }
                finally
                {
                    clone.release();
                    saveSemaphore.release();
                }
            }
        }).start();
    }

    // example usage: saveMatToDisk(input, "EOCV_frame");
    public void saveMatToDisk(Mat mat, final String filename)
    {
        saveMatToDiskFullPath(mat, String.format("%s/%s.png", defaultSavePath, filename));
    }
}
