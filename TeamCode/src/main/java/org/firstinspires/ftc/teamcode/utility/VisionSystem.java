package org.firstinspires.ftc.teamcode.utility;


import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.vision.ColorDetect;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

public class VisionSystem {

    private final HardwareMap hardwareMap;
    private final CameraName switchableCamera;
    protected VisionPortal visionPortal;
    private AprilTagProcessor aprilTagProcessor;

    // our HSV pipeline that detects custom game pieces.
   // private HSVSaturationProcessor gamePieceProcessor;
//    private ColorDetect  colorDetectProcessor;
    private ColorDetect gamePieceProcessor;

    private WebcamName frontCam;
    private WebcamName rearCam;

    private Telemetry telemetry;

    private VisionProcessorMode currentMode = VisionProcessorMode.NONE;

    static final int STREAM_WIDTH = 640; // modify for your camera
    static final int STREAM_HEIGHT = 480; // modify for your camera

    public VisionSystem(HardwareMap hMap,Telemetry telemetry){
        this.hardwareMap = hMap;
        this.telemetry = telemetry;

        // Build the AprilTag processor
        // set parameters of AprilTagProcessor, then use Builder to build
        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                .build();

        // set apriltag resolution decimation factor
        aprilTagProcessor.setDecimation(2);

        frontCam = hardwareMap.get(WebcamName.class, "gge_cam");
        rearCam = hardwareMap.get(WebcamName.class, "gge_backup_cam");
        switchableCamera = ClassFactory.getInstance()
                .getCameraManager()
                .nameForSwitchableCamera(frontCam,rearCam);

        // create our HSV processor
        gamePieceProcessor = new ColorDetect(); //HSVSaturationProcessor();
//        colorDetectProcessor = new ColorDetect();

        // Todo:  add a pixel tensorflow processor here

        // Build the vision portal
        // set parameters,then use vision builder.
        visionPortal = new VisionPortal.Builder()
                .setCamera(switchableCamera)
                .addProcessors(aprilTagProcessor,gamePieceProcessor) // add all the processors here
                //.setCameraResolution(new Size(640, 480))
                .setCameraResolution(new Size(STREAM_WIDTH,STREAM_HEIGHT))
                .enableLiveView(false)
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .build();

    }

    public boolean camerasReady(){
        return( visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING);
    }

    public VisionProcessorMode setVisionProcessingMode(VisionProcessorMode newMode){

        // if we go through this switch and are unable to set a new mode, the mode
        // will be
        switch(newMode){
            case CLAW_CAMERA:
                switchCamera(frontCam, telemetry);

                visionPortal.setProcessorEnabled(aprilTagProcessor,false);
                visionPortal.setProcessorEnabled(gamePieceProcessor,true);

                this.currentMode = newMode;
                break;
            case REAR_CAMERA_BACKDROP_APRIL_TAG:
                switchCamera(rearCam, telemetry);

                visionPortal.setProcessorEnabled(aprilTagProcessor,true);
                visionPortal.setProcessorEnabled(gamePieceProcessor,false);

                this.currentMode = newMode;
                break;
            case FRONT_CAMERA_PIXEL_GRAB:
                // will want to enable the Tensor Flow processor here
                break;
            case NONE:
            default:
                // The processing mode didn't change, the value of this.current mode doesn't change

        }

        return currentMode;
    }

    public void stopLiveView()
    {
        visionPortal.stopLiveView();
    }

    public void resumeLiveView(){
        visionPortal.resumeLiveView();
    }

    private void switchCamera(WebcamName desiredWebCam, Telemetry telemetry) {
        while (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
        }
        if (visionPortal.getActiveCamera() != desiredWebCam) {
            visionPortal.setActiveCamera(desiredWebCam);
            while (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
                telemetry.addData("Camera State: ", visionPortal.getCameraState());
                telemetry.update();
            }
        }
    }

//
//    public double getLeftSpikeSaturation() {
//        return gamePieceProcessor.getLeftSpikeSaturation();
//    }
//
//    public double getCenterSpikeSaturation() {
//        return gamePieceProcessor.getCenterSpikeSaturation();
//    }
//
//    public double getRightSpikeSaturation() {
//        return gamePieceProcessor.getRightSpikeSaturation();
//    }

    // done processing  let's clean up.  So far the only thing to do is close the vision Portal
    public void close() {
        visionPortal.close();
    }

    public List<AprilTagDetection> getDetections() {
        return aprilTagProcessor.getDetections();
    }
}