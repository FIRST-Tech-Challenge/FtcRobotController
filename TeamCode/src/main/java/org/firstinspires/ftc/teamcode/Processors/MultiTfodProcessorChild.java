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

import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.external.tfod.CameraInformation;
import org.firstinspires.ftc.robotcore.external.tfod.CanvasAnnotator;
import org.firstinspires.ftc.robotcore.external.tfod.FrameConsumer;
import org.firstinspires.ftc.robotcore.external.tfod.FrameGenerator;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Mat;

public class MultiTfodProcessorChild implements VisionProcessor, FrameGenerator {
    protected int width;
    protected int height;
    protected Mat currentFrame;
    protected Mat outputFrame;
    protected float fx, fy = 100;
    protected boolean frameRead;
    protected FrameConsumer frameConsumer;

    public Mat getCameraMat() {
        return this.currentFrame;
    }

    public void setOutputFrame(Mat frame) {
        this.outputFrame = frame;
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        this.height = height;
        this.width = width;
        if (calibration != null) {
            fx = calibration.focalLengthX;
            fy = calibration.focalLengthY;
        }
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        if(this.currentFrame != null) this.currentFrame.release();
        this.currentFrame = frame.clone();
        return frame;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        ((CanvasAnnotator) userContext).draw(canvas, onscreenWidth, onscreenHeight, scaleBmpPxToCanvasPx, scaleCanvasDensity);
    }

    @Override
    public CameraInformation getCameraInformation() {
        return new CameraInformation(width, height, 0, fx, fy);
    }

    @Override
    public void setFrameConsumer(FrameConsumer frameConsumer) {
        this.frameConsumer = frameConsumer;
    }
}

