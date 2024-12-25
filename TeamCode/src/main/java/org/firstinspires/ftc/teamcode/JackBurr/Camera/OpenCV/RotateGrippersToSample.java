package org.firstinspires.ftc.teamcode.JackBurr.Camera.OpenCV;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.opencv.core.Point;
import java.util.List;
@TeleOp
public class RotateGrippersToSample extends OpMode {
    SampleDetectorToolkit toolkit;
    SampleDetectorVisionPortalToolkit visionToolkit;
    ColorBlobLocatorProcessor red;
    ColorBlobLocatorProcessor neutral;
    ColorBlobLocatorProcessor blue;
    List<ColorBlobLocatorProcessor> processorList;
    List<ColorBlobLocatorProcessor.Blob> redList;
    List<ColorBlobLocatorProcessor.Blob> neutralList;
    List<ColorBlobLocatorProcessor.Blob> blueList;
    List<SampleDetection> masterList;
    VisionPortal portal;
    int MIN_AREA = 50;
    int MAX_AREA = 20000;
    Point center;
    SampleDetection detection;
    @Override
    public void init() {
        telemetry.setMsTransmissionInterval(50);
        toolkit = new SampleDetectorToolkit();
        red = toolkit.getNewProcessor(ColorRange.RED);
        neutral = toolkit.getNewProcessor(ColorRange.YELLOW);
        blue = toolkit.getNewProcessor(ColorRange.BLUE);
        processorList.add(red);
        processorList.add(neutral);
        processorList.add(blue);
        portal = visionToolkit.createVisionPortal(hardwareMap, processorList, "Webcam 1");
        center = toolkit.getCenter(320, 240);
    }

    @Override
    public void init_loop(){
        redList = toolkit.filterByArea(MIN_AREA, MAX_AREA, red.getBlobs());
        neutralList = toolkit.filterByArea(MIN_AREA, MAX_AREA, neutral.getBlobs());
        blueList = toolkit.filterByArea(MIN_AREA, MAX_AREA, blue.getBlobs());
        masterList = toolkit.addToSampleDetectionList(masterList, ColorRange.RED, redList);
        masterList = toolkit.addToSampleDetectionList(masterList, ColorRange.YELLOW, neutralList);
        masterList = toolkit.addToSampleDetectionList(masterList, ColorRange.BLUE, blueList);
        detection = toolkit.findClosestSample(center, masterList);
        telemetry.addLine("Detected " + detection.color + " sample/specimen:");
        telemetry.addLine("\t X Position: " + detection.x);
        telemetry.addLine("\t Y Position: " + detection.y);
        telemetry.addLine("\t Width: " + detection.width);
        telemetry.addLine("\t Height: " + detection.height);
        telemetry.addLine("\t Angle: " + detection.angle);

    }

    @Override
    public void loop() {
        portal.stopStreaming();
    }
}
