package org.firstinspires.ftc.teamcode.hardwareSystems;

import android.util.Size;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.firstinspires.ftc.vision.opencv.PredominantColorProcessor;
import org.firstinspires.ftc.vision.VisionPortal;

public class Webcam {
    private final PredominantColorProcessor COLOR_PROCESSOR;
    private final VisionPortal VISION_PORTAL;
    private static final int RESOLUTION_WIDTH = 100;
    private static final int RESOLUTION_HEIGHT = 100;

    public Webcam(WebcamName webcam) {
        COLOR_PROCESSOR = new PredominantColorProcessor.Builder()
                .setRoi(ImageRegion.asUnityCenterCoordinates(-0.5, 0.5, 0.5, -0.5))
                .setSwatches(
                        PredominantColorProcessor.Swatch.RED,
                        PredominantColorProcessor.Swatch.BLUE,
                        PredominantColorProcessor.Swatch.YELLOW,
                        PredominantColorProcessor.Swatch.BLACK,
                        PredominantColorProcessor.Swatch.WHITE)
                .build();

        VISION_PORTAL = new VisionPortal.Builder()
                .addProcessor(COLOR_PROCESSOR)
                .setCamera(webcam)
                .setCameraResolution(new Size(RESOLUTION_WIDTH, RESOLUTION_HEIGHT))
                .build();
    }

    public PredominantColorProcessor getColorProcessor() {
        return COLOR_PROCESSOR;
    }

    public VisionPortal getVisionPortal() {
        return VISION_PORTAL;
    }
}