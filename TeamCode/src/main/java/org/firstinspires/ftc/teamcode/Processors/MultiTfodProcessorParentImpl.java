/*
 * Copyright (c) 2023 FIRST
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to
 * endorse or promote products derived from this software without specific prior
 * written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
 * TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.Processors;

import android.graphics.Bitmap;
import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.tfod.CameraInformation;
import org.firstinspires.ftc.robotcore.external.tfod.CanvasAnnotator;
import org.firstinspires.ftc.robotcore.external.tfod.FrameConsumer;
import org.firstinspires.ftc.robotcore.external.tfod.FrameGenerator;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.TfodParameters;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Size;

import java.util.ArrayList;
import java.util.List;

class MultiTfodProcessorParentImpl extends MultiTfodProcessorParent implements FrameGenerator
{
    protected final TfodParameters parameters;
    protected final TFObjectDetector tfObjectDetector;
    protected final Object frameConsumerLock = new Object();
    protected FrameConsumer frameConsumer;
    protected Bitmap bitmap;
    protected int width;
    protected int height;
    protected float fx, fy = 100; // dummy
    protected List<MultiTfodProcessorChild> children;

    public MultiTfodProcessorParentImpl(TfodParameters parameters, List<MultiTfodProcessorChild> children)
    {
        this.parameters = parameters;
        this.children = children;
        tfObjectDetector = ClassFactory.getInstance().createTFObjectDetector(parameters, this);
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration)
    {
        this.width = width;
        this.height = height;

        if (calibration != null)
        {
            fx = calibration.focalLengthX;
            fy = calibration.focalLengthY;
        }

        tfObjectDetector.activate();
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos)
    {
        FrameConsumer frameConsumerSafe;
        Bitmap bitmapSafe;

        synchronized (frameConsumerLock)
        {
            if (frameConsumer == null)
            {
                return null;
            }
            frameConsumerSafe = frameConsumer;
            bitmapSafe = bitmap;
        }

        Mat mat = new Mat(new Size(640, 240), frame.type());

        List<Mat> mats = new ArrayList<>();
        mats.add(frame);

        if (this.children != null) {
            for (MultiTfodProcessorChild child : this.children) {
                Mat childMat = child.getCameraMat();
                if (childMat != null) {
                    mats.add(childMat);
                    childMat.release();
                }
            }
            try {
                Core.hconcat(mats, mat);
            } catch (Error e) {
                mat.release();
                mat = frame.clone();
            }
            boolean allSame = true;
            for (Mat matTwo: mats) {
                if (matTwo != null && (matTwo.type() != frame.type()) && (matTwo.rows() != frame.rows())) allSame = false;
            }
            assert(allSame);
        } else {
            mat.release();
            mat = frame.clone();
        }

        Utils.matToBitmap(mat, bitmapSafe);
//        CanvasAnnotator out = frameConsumerSafe.processFrame();
        return frameConsumerSafe.processFrame();
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext)
    {
        if (userContext != null)
        {
            ((CanvasAnnotator) userContext).draw(canvas, onscreenWidth, onscreenHeight, scaleBmpPxToCanvasPx, scaleCanvasDensity);
        }
    }

    @Override
    public CameraInformation getCameraInformation()
    {
        return new CameraInformation(width, height, 0, fx, fy);
    }

    @Override
    public void setFrameConsumer(FrameConsumer frameConsumer)
    {
        synchronized (frameConsumerLock)
        {
            this.frameConsumer = frameConsumer;

            if (frameConsumer != null)
            {
                bitmap = Bitmap.createBitmap(width, height, Bitmap.Config.ARGB_8888);
                frameConsumer.init(bitmap);
            }
        }
    }

    @Override
    public void setMinResultConfidence(float minResultConfidence) {
        tfObjectDetector.setMinResultConfidence(minResultConfidence);
    }

    @Override
    public void setClippingMargins(int left, int top, int right, int bottom)
    {
        tfObjectDetector.setClippingMargins(left, top, right, bottom);
    }

    @Override
    public void setZoom(double magnification)
    {
        tfObjectDetector.setZoom(magnification);
    }

    @Override
    public List<Recognition> getRecognitions()
    {
        return tfObjectDetector.getRecognitions();
    }

    @Override
    public List<Recognition> getFreshRecognitions()
    {
        return tfObjectDetector.getUpdatedRecognitions();
    }

    @Override
    public void shutdown()
    {
        tfObjectDetector.shutdown();
    }
}
