package org.firstinspires.ftc.teamcode.tatooine.SubSystem.camera;

import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.opencv.core.Mat;

import java.util.Collections;
import java.util.List;

public class BlobProcessorEX extends ColorBlobLocatorProcessor {


    @Override
    public void init(int width, int height, CameraCalibration calibration) {

    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        return
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

    }

    @Override
    public void addFilter(BlobFilter filter) {

    }

    @Override
    public void removeFilter(BlobFilter filter) {

    }

    @Override
    public void removeAllFilters() {

    }

    @Override
    public void setSort(BlobSort sort) {

    }

    @Override
    public List<Blob> getBlobs() {
        return Collections.emptyList();
    }
}
