package org.rustlib.vision;

import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;

public interface FrameProcessor extends VisionProcessor {
    @Override
    default void init(int width, int height, CameraCalibration calibration) {

    }

    @Override
    default void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

    }
}
