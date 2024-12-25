package org.firstinspires.ftc.teamcode.JackBurr.Camera.OpenCV;


import android.graphics.HardwareRenderer;
import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ImageRegion;

import java.util.ArrayList;
import java.util.List;

public class SampleDetectorVisionPortalToolkit {

    public ColorBlobLocatorProcessor createNewProcessor(ColorRange color) {
        ColorBlobLocatorProcessor locatorProcessor;
        locatorProcessor = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(color)
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                .setRoi(ImageRegion.entireFrame())
                .setDrawContours(false)
                .setBlurSize(5)
                .setDilateSize(5)
                .build();
        return locatorProcessor;
    }

    public VisionPortal createVisionPortal(HardwareMap hardwareMap, List<ColorBlobLocatorProcessor> processorsList, String webcamName){
        VisionPortal.Builder portalBuilder;
        portalBuilder = new VisionPortal.Builder()
                .setCameraResolution(new Size(320, 240))
                .setCamera(hardwareMap.get(WebcamName.class, webcamName));
        for(VisionProcessor processor : processorsList) {
            portalBuilder.addProcessor(processor);
        }
        return portalBuilder.build();
    }

    public VisionPortal createVisionPortal(HardwareMap hardwareMap, VisionProcessor processor, String webcamName){
        VisionPortal portal;
        portal = new VisionPortal.Builder()
                    .addProcessor(processor)
                    .setCameraResolution(new Size(320, 240))
                    .setCamera(hardwareMap.get(WebcamName.class, webcamName))
                    .build();
            return portal;
        }

    public void setSort(ColorBlobLocatorProcessor processor, ColorBlobLocatorProcessor.BlobSort sort){
        processor.setSort(sort);
    }

    public void getBlobs(ColorBlobLocatorProcessor processor){
        processor.getBlobs();
    }



}
