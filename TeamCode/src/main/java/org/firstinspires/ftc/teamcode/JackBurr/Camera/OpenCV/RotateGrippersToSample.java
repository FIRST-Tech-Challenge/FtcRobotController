package org.firstinspires.ftc.teamcode.JackBurr.Camera.OpenCV;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.opencv.core.Point;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;
import java.util.List;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
@Disabled
@TeleOp
public class RotateGrippersToSample extends OpMode {
    public SampleDetectorToolkit toolkit;
    public SampleDetectorVisionPortalToolkit visionToolkit;
    public ColorBlobLocatorProcessor red;
    public ColorBlobLocatorProcessor neutral;
    public ColorBlobLocatorProcessor blue;
    public List<ColorBlobLocatorProcessor> processorList = new ArrayList<>();
    public List<ColorBlobLocatorProcessor.Blob> redList = new ArrayList<>();
    public List<ColorBlobLocatorProcessor.Blob> neutralList = new ArrayList<>();
    public List<ColorBlobLocatorProcessor.Blob> blueList = new ArrayList<>();
    public List<SampleDetection> masterList = new ArrayList<>();
    public VisionPortal portal;
    public int MIN_AREA = 2000;
    public int MAX_AREA = 20000;
    public Point center;
    public SampleDetection detection;
    @Override
    public void init() {
        telemetry.setMsTransmissionInterval(50);
        toolkit = new SampleDetectorToolkit(hardwareMap);
        visionToolkit = new SampleDetectorVisionPortalToolkit(hardwareMap);
        red = toolkit.getNewProcessor(ColorRange.RED, MIN_AREA, MAX_AREA);
        neutral = toolkit.getNewProcessor(ColorRange.YELLOW,  MIN_AREA, MAX_AREA);
        blue = toolkit.getNewProcessor(ColorRange.BLUE,  MIN_AREA, MAX_AREA);
       // processorList.add(red);
        processorList.add(neutral);
        //processorList.add(blue);
        portal = visionToolkit.createVisionPortal(hardwareMap, processorList, "Webcam 1");
        center = toolkit.getCenter(320, 240);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        OpenCvWebcam camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        FtcDashboard.getInstance().startCameraStream(camera, 0);
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
        telemetry.addLine("Detected " + toolkit.getColorName(detection) + " sample/specimen:");
        telemetry.addLine("\t X Position: " + detection.x);
        telemetry.addLine("\t Y Position: " + detection.y);
        telemetry.addLine("\t Width: " + detection.width);
        telemetry.addLine("\t Height: " + detection.height);
        telemetry.addLine("\t Angle: " + detection.angle);
        telemetry.addLine("\t Rotation Needed: " + toolkit.findNeededRotationDegrees(detection.boxFit));

    }

    @Override
    public void loop() {
        portal.stopStreaming();
    }
}
