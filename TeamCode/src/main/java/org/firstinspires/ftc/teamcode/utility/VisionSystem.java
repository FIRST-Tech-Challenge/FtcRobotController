package org.firstinspires.ftc.teamcode.utility;


import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.vision.pipeline.HSVSaturationProcessor;
import org.firstinspires.ftc.teamcode.vision.util.FieldPosition;
import org.firstinspires.ftc.teamcode.vision.util.SpikePosition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

public class VisionSystem {
    private final HardwareMap hardwareMap;
    protected VisionPortal visionPortal;
    private AprilTagProcessor aprilTagProcessor;

    // our HSV pipeline that detects custom game pieces.
    private HSVSaturationProcessor gamePieceProcessor;

    private WebcamName frontCam;
    private WebcamName rearCam;

    private Telemetry telemetry;

    private VisionProcessorMode currentMode = VisionProcessorMode.NONE;
    
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
        aprilTagProcessor.setDecimation(3);

        frontCam = hardwareMap.get(WebcamName.class, "gge_cam");
        rearCam = hardwareMap.get(WebcamName.class, "gge_backup_cam");
        CameraName switchableCamera = ClassFactory.getInstance()
                .getCameraManager()
                .nameForSwitchableCamera(frontCam,rearCam);

        // create our HSV processor
        gamePieceProcessor = new HSVSaturationProcessor();

        // Todo:  add a pixel tensorflow processor here

        // Build the vision portal
        // set parameters,then use vision builder.
        visionPortal = new VisionPortal.Builder()
                .setCamera(switchableCamera)
                .addProcessors(aprilTagProcessor,gamePieceProcessor) // add all the processors here
                .setCameraResolution(new Size(640, 480))
                //.setCameraResolution(new Size(1280,720))
                .enableLiveView(false)
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .build();

    }
    public VisionProcessorMode setVisionProcessingMode(VisionProcessorMode newMode){

        // if we go through this switch and are unable to set a new mode, the mode
        // will be
        switch(newMode){
            case FRONT_CAMERA_GAMEPIECE:
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
                break;
            case NONE:

        }


        return currentMode;
    }

    private void switchCamera(WebcamName frontCam, Telemetry telemetry) {
        if (visionPortal.getActiveCamera() != frontCam) {
            visionPortal.setActiveCamera(frontCam);
            while (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
                telemetry.addData("Camera State: ", visionPortal.getCameraState());
                telemetry.update();
                //Thread.sleep(20);  may need this.  The while should just keep going
            }
        }
        telemetry.addData("Camera State: ",visionPortal.getCameraState());
        telemetry.update();
    }

    public void setFieldPosition(FieldPosition fPos) {
        gamePieceProcessor.setFieldPosition(fPos);
    }

    public SpikePosition getSpikePosition() {
        return gamePieceProcessor.getSpikePos();
    }

    public double getLeftSpikeSaturation() {
        return gamePieceProcessor.getLeftSpikeSaturation();
    }

    public double getCenterSpikeSaturation() {
        return gamePieceProcessor.getCenterSpikeSaturation();
    }

    public double getRightSpikeSaturation() {
        return gamePieceProcessor.getRightSpikeSaturation();
    }

    // done processing  let's clean up.  So far the only thing to do is close the vision Portal
    public void close() {
        visionPortal.close();
    }

    public List<AprilTagDetection> getDetections() {
         return aprilTagProcessor.getDetections();
    }
}
