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
    public HardwareMap hardwareMap;
    public SampleDetectorVisionPortalToolkit(HardwareMap hardwareMap){
        this.hardwareMap = hardwareMap;
    }

    public ColorBlobLocatorProcessor createNewProcessor(ColorRange color, int minArea, int maxArea) {
        ColorBlobLocatorProcessor locatorProcessor;
        ColorBlobLocatorProcessor.BlobFilter myAreaFilter = new ColorBlobLocatorProcessor.BlobFilter(ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA, minArea, maxArea);
        locatorProcessor = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(color)
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                .setRoi(ImageRegion.entireFrame())
                .setDrawContours(false)
                .setDilateSize(0)
                .setErodeSize(3)
                .build();
        locatorProcessor.addFilter(myAreaFilter);
        return locatorProcessor;
    }

    public VisionPortal createVisionPortal(HardwareMap hardwareMap, List<ColorBlobLocatorProcessor> processorsList, String webcamName){
        VisionPortal.Builder portalBuilder;
        portalBuilder = new VisionPortal.Builder()
                .setCameraResolution(new Size(1280, 720))
                .setCamera(hardwareMap.get(WebcamName.class, webcamName))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG);
        for(VisionProcessor processor : processorsList) {
            portalBuilder.addProcessor(processor);
        }
        portalBuilder.setLiveViewContainerId(
                hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName()));
        return portalBuilder.build();
    }

    public VisionPortal createVisionPortal(VisionProcessor processor, String webcamName){
        VisionPortal portal;
        portal = new VisionPortal.Builder()
                    .addProcessor(processor)
                    .setCameraResolution(new Size(1280, 960))
                    .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
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
