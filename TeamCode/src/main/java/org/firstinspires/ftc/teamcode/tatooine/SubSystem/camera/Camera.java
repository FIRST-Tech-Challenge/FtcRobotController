package org.firstinspires.ftc.teamcode.tatooine.SubSystem.camera;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ImageRegion;

public class Camera {

    private Camera camera;

    private boolean isRed = false;

    private boolean isSpecimen = false;

    private boolean IS_DEBUG = false;

    private Telemetry telemetry;

    public Camera(OpMode opMode,boolean isRed, boolean isSpecimen, Telemetry telemetry) {

        this.IS_DEBUG = IS_DEBUG;
        this.telemetry = telemetry;
        this.isRed = isRed;
        this.isSpecimen = isSpecimen;

        camera = opMode.hardwareMap.get(Camera.class, "camera");

        if (IS_DEBUG) {
            telemetry.addData("CameraConstructor", true);
        }


        ColorBlobLocatorProcessor colorLocator = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(ColorRange.BLUE)         // use a predefined color match
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)    // exclude blobs inside blobs
                .setRoi(ImageRegion.asUnityCenterCoordinates(-0.5, 0.5, 0.5, -0.5))  // search central 1/4 of camera view
                .setDrawContours(true)                        // Show contours on the Stream Preview
                .setBlurSize(5)                               // Smooth the transitions between different colors in image
                .build();


        VisionPortal portal = new VisionPortal.Builder()
                .addProcessor(colorLocator)
                .setCameraResolution(new Size(320, 240))
                .setCamera(opMode.hardwareMap.get(WebcamName.class, "Webcam 1"))
                .build();




    }

    public void setCamera(Camera camera) {
        this.camera = camera;
    }

    public void setRed(boolean red) {
        isRed = red;
    }

    public void setSpecimen(boolean specimen) {
        isSpecimen = specimen;
    }

    public void setIS_DEBUG(boolean IS_DEBUG) {
        this.IS_DEBUG = IS_DEBUG;
    }

    public boolean isIS_DEBUG() {
        return IS_DEBUG;
    }

    public Camera getCamera() {
        return camera;
    }

    public boolean isRed() {
        return isRed;
    }

    public boolean isSpecimen() {
        return isSpecimen;
    }
}
