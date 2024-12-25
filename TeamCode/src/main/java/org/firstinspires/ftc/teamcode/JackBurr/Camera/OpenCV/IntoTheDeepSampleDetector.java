package org.firstinspires.ftc.teamcode.JackBurr.Camera.OpenCV;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.opencv.core.Rect;
import org.opencv.core.RotatedRect;

import java.util.Iterator;
import java.util.List;

@TeleOp
public class IntoTheDeepSampleDetector extends OpMode {
    ColorBlobLocatorProcessor blueLocator;
    ColorBlobLocatorProcessor neutralLocator;
    ColorBlobLocatorProcessor redLocator;
    VisionPortal processor;
    List<ColorBlobLocatorProcessor.Blob> redBlobs;
    List<ColorBlobLocatorProcessor.Blob> blueBlobs;
    List<ColorBlobLocatorProcessor.Blob> neutralBlobs;

    @Override
    public void init() {
         blueLocator = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(ColorRange.BLUE)         // use a predefined color match
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)    // exclude blobs inside blobs
                .setRoi(ImageRegion.entireFrame())  // search all of camera view
                .setDrawContours(false)                        // Show contours on the Stream Preview
                .setBlurSize(5)                             // Smooth the transitions between different colors in image
                .setDilateSize(5)
                .build();

        neutralLocator = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(ColorRange.YELLOW)// use a predefined color match
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)    // exclude blobs inside blobs
                .setRoi(ImageRegion.entireFrame())  // search all of camera view
                .setDrawContours(false)                        // Show contours on the Stream Preview
                .setBlurSize(5) // Smooth the transitions between different colors in image
                .setDilateSize(5)
                .build();
        redLocator = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(ColorRange.RED)// use a predefined color match
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)    // exclude blobs inside blobs
                .setRoi(ImageRegion.entireFrame())  // search all of camera view
                .setDrawContours(false)                        // Show contours on the Stream Preview
                .setBlurSize(5) // Smooth the transitions between different colors in image
                .setDilateSize(5)
                .build();

        processor = new VisionPortal.Builder()
                .addProcessor(neutralLocator)
                .addProcessor(redLocator)
                .addProcessor(blueLocator)
                .setCameraResolution(new Size(320, 240))
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .build();
    }

    @Override
    public void init_loop(){
        redBlobs = redLocator.getBlobs();
        blueBlobs = blueLocator.getBlobs();
        neutralBlobs = neutralLocator.getBlobs();

        Iterator<ColorBlobLocatorProcessor.Blob> redIterator = redBlobs.iterator();
        while (redIterator.hasNext()) {
            ColorBlobLocatorProcessor.Blob b = redIterator.next();
            RotatedRect boxFit = b.getBoxFit();
            if (boxFit.size.height > 40 && boxFit.size.width > 40) {
                telemetry.addLine("Red" + " sample of size " + boxFit.size + " at " + boxFit.center + " with angle " + boxFit.angle);
            }
            else {
                redIterator.remove();
            }
        }

        Iterator<ColorBlobLocatorProcessor.Blob> neutralIterator = neutralBlobs.iterator();
        while (neutralIterator.hasNext()) {
            ColorBlobLocatorProcessor.Blob b = neutralIterator.next();
            RotatedRect boxFit = b.getBoxFit();
            if (boxFit.size.height > 40 && boxFit.size.width > 40) {
                telemetry.addLine("Neutral" + " sample of size " + boxFit.size + " at " + boxFit.center + " with angle " + boxFit.angle);
            }
            else {
                neutralIterator.remove();
            }
        }
        Iterator<ColorBlobLocatorProcessor.Blob> blueIterator = blueBlobs.iterator();
        while (blueIterator.hasNext()) {
            ColorBlobLocatorProcessor.Blob b = blueIterator.next();
            RotatedRect boxFit = b.getBoxFit();
            SampleDetection sample = new SampleDetection(ColorRange.BLUE, boxFit);
            if (sample.height > 40 && sample.width > 40) {
                telemetry.addLine("Blue" + " sample of size " + boxFit.size + " at " + boxFit.center + " with angle " + boxFit.angle);
            }
            else {
                    blueIterator.remove();
            }
        }
    }

    @Override
    public void loop() {

    }
}
