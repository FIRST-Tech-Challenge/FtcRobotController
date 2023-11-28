package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraControls;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.objects.HydraOpMode;
import org.firstinspires.ftc.teamcode.types.HydraAprilTagPose;
import org.firstinspires.ftc.teamcode.types.HydraObjectLocations;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.ArrayList;
import java.util.List;

public class HydraObjectDetect {
    private final VisionPortal myVisionPortal;
    private final TfodProcessor myTfodProcessor;
    protected final float cXvalueForLeftToCenterObject = 200;
    private HydraOpMode mOp;
    private final AprilTagProcessor mAprilTagProcessor;

    public HydraObjectDetect(HydraOpMode op, String modelFilename) {
        mOp = op;
        /*
         * Build the tensor flow object detection processor
         */
        TfodProcessor.Builder myTfodProcessorBuilder;
        // First, create a TfodProcessor.Builder.
        myTfodProcessorBuilder = new TfodProcessor.Builder();
        // Set the name of the file where the model can be found.
        myTfodProcessorBuilder.setModelFileName(modelFilename);
        // Set the full ordered list of labels the model is trained to recognize.
        myTfodProcessorBuilder.setModelLabels(JavaUtil.createListWith("prop"));
        // Set the aspect ratio for the images used when the model was created.
        myTfodProcessorBuilder.setModelAspectRatio((double)4 / 3);
        // Create a TfodProcessor by calling build.
        myTfodProcessor = myTfodProcessorBuilder.build();
        /*
         * Build the april tag detection processor
         */
        AprilTagProcessor.Builder myAprilTagProcessorBuilder = new AprilTagProcessor.Builder();
        // Get the AprilTagLibrary for the current season and set the library
        myAprilTagProcessorBuilder.setTagLibrary(AprilTagGameDatabase.getCurrentGameTagLibrary());
        // Build the processor
        mAprilTagProcessor = myAprilTagProcessorBuilder.build();
        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        mAprilTagProcessor.setDecimation(3);
        /*
         * Create the vision portal that handles all of the processing
         */
        // Next, create a VisionPortal.Builder and set attributes related to the camera.
        VisionPortal.Builder myVisionPortalBuilder = new VisionPortal.Builder();
        // Use a webcam.
        myVisionPortalBuilder.setCamera(mOp.mHardwareMap.get(WebcamName.class, "Webcam 1"));
        // Add myTfodProcessor to the VisionPortal.Builder.
        myVisionPortalBuilder.addProcessor(myTfodProcessor);
        // Add april tag processor to the portal
        myVisionPortalBuilder.addProcessor(mAprilTagProcessor);
        // Create a VisionPortal by calling build.
        myVisionPortal = myVisionPortalBuilder.build();
    }

    /**
     * Finds and returns the location of an object using the provided model
     * @return unknown if we saw nothing, left or center if we detected an object
     */
    public HydraObjectLocations GetObjectLocation(boolean trueForRed) {
        HydraObjectLocations detectedLocation = HydraObjectLocations.ObjLocUnknown;
        Recognition myTfodRecognition;
        VisionPortal.CameraState camState = myVisionPortal.getCameraState();
        // Get a list of recognitions from TFOD.
        mOp.mTelemetry.addData("Camera State", camState);
        List<Recognition> myTfodRecognitions = myTfodProcessor.getRecognitions();
        mOp.mTelemetry.addData("# Objects Detected", JavaUtil.listLength(myTfodRecognitions));
        if (mOp.mObjLogger != null) {
            ZeroOutLogger();
            mOp.mObjLogger.camState.set(camState.toString());
            mOp.mObjLogger.numObjDet.set(JavaUtil.listLength(myTfodRecognitions));
            if (JavaUtil.listIsEmpty(myTfodRecognitions)) {
                mOp.mObjLogger.writeLine();
            }
        }
        // keep the detection with the highest confidence
        double highestConfidence = 0;
        // Iterate through list and call a function to display info for each recognized object.
        for (Recognition myTfodRecognition_item : myTfodRecognitions) {
            myTfodRecognition = myTfodRecognition_item;
            // Display info about the recognition.
            mOp.mTelemetry.addLine("");
            // Display label and confidence.
            // Display the label and confidence for the recognition.
            mOp.mTelemetry.addData("Image", myTfodRecognition.getLabel() + " (" +
                    JavaUtil.formatNumber(myTfodRecognition.getConfidence() * 100, 0) +
                    " % Conf.)");
            // Display position.
            float x = (myTfodRecognition.getLeft() + myTfodRecognition.getRight()) / 2;
            float y = (myTfodRecognition.getTop() + myTfodRecognition.getBottom()) / 2;
            // we will keep the detected object that has the highest confidence
            double confidence = myTfodRecognition.getConfidence();
            // Display the position of the center of the detection boundary for the recognition
            mOp.mTelemetry.addData("- Position", JavaUtil.formatNumber(x, 0) +
                    ", " + JavaUtil.formatNumber(y, 0));
            if (mOp.mObjLogger != null) {
                mOp.mObjLogger.objX.set(x);
                mOp.mObjLogger.objY.set(y);
                mOp.mObjLogger.objConf.set(confidence);
                mOp.mObjLogger.writeLine();
            }
            if (confidence >= highestConfidence) {
                highestConfidence = confidence;
                if (x < cXvalueForLeftToCenterObject) {
                    if (trueForRed) {
                        detectedLocation = HydraObjectLocations.ObjLocRedLeftSpike;
                    } else {
                        detectedLocation = HydraObjectLocations.ObjLocBlueLeftSpike;
                    }
                } else {
                    if (trueForRed) {
                        detectedLocation = HydraObjectLocations.ObjLocRedCenterSpike;
                    } else {
                        detectedLocation = HydraObjectLocations.ObjLocBlueCenterSpike;
                    }
                }
                // uncomment this break if you want to just take the first one like we used to
                // break;
            }
        }
        return detectedLocation;
    }

    public HydraAprilTagPose FindAprilTag(HydraObjectLocations objLocToFind) {
        List<AprilTagDetection> aprilTagDetections = mAprilTagProcessor.getDetections();
        VisionPortal.CameraState camState = myVisionPortal.getCameraState();
        mOp.mTelemetry.addData("Camera State", camState);
        if (mOp.mObjLogger != null) {
            ZeroOutLogger();
            mOp.mObjLogger.camState.set(camState.toString());
            mOp.mObjLogger.numAprilDet.set(JavaUtil.listLength(aprilTagDetections));
            if (JavaUtil.listIsEmpty(aprilTagDetections)) {
                mOp.mObjLogger.writeLine();
            }
        }
        HydraAprilTagPose ret = null;
        for (AprilTagDetection aprilTag : aprilTagDetections) {
            if (aprilTag.metadata != null) {
                mOp.mTelemetry.addData("April Id", aprilTag.id);
                mOp.mTelemetry.addData("April Name", aprilTag.metadata.name);
                mOp.mTelemetry.addData("April Range", aprilTag.ftcPose.range);
                mOp.mTelemetry.addData("April Bearing", aprilTag.ftcPose.bearing);
                mOp.mTelemetry.addData("April Yaw", aprilTag.ftcPose.yaw);
                HydraAprilTagPose pose = new HydraAprilTagPose();
                pose.mPoseX = aprilTag.ftcPose.x;
                pose.mPoseY = aprilTag.ftcPose.y;
                pose.mPoseZ = aprilTag.ftcPose.z;
                pose.mPosePitch = aprilTag.ftcPose.pitch;
                pose.mPoseRoll = aprilTag.ftcPose.roll;
                pose.mPoseYaw = aprilTag.ftcPose.yaw;
                pose.mPoseRange = aprilTag.ftcPose.range;
                pose.mPoseBearing = aprilTag.ftcPose.bearing;
                pose.mPoseElevation = aprilTag.ftcPose.elevation;
                if (pose.mId == GetAprilIdForLocation(objLocToFind)) {
                    ret = pose;
                }
                if (mOp.mObjLogger != null) {
                    mOp.mObjLogger.aprilName.set(aprilTag.metadata.name);
                    mOp.mObjLogger.aprilRange.set(aprilTag.ftcPose.range);
                    mOp.mObjLogger.aprilBearing.set(aprilTag.ftcPose.bearing);
                    mOp.mObjLogger.aprilYaw.set(aprilTag.ftcPose.yaw);
                    mOp.mObjLogger.writeLine();
                }
            }
        }
        return ret;
    }

    private int GetAprilIdForLocation(HydraObjectLocations objLocToFind) {
        switch (objLocToFind) {
            case ObjLocBlueLeftSpike:
                return 1;
            case ObjLocBlueCenterSpike:
                return 2;
            case ObjLocBlueRightSpike:
                return 3;
            case ObjLocRedLeftSpike:
                return 4;
            case ObjLocRedCenterSpike:
                return 5;
            case ObjLocRedRightSpike:
                return 6;
            default:
                return -1;
        }
    }

    public void SetObjDetectEnabled(boolean enabled) {
        myVisionPortal.setProcessorEnabled(myTfodProcessor, enabled);
        if (!enabled && mOp.mObjLogger != null) {
            mOp.mObjLogger.camState.set(myVisionPortal.getCameraState().toString());
            ZeroOutLogger();
            mOp.mObjLogger.writeLine();
        }
    }

    public void SetAprilDetectEnabled(boolean enabled) {
        myVisionPortal.setProcessorEnabled(mAprilTagProcessor, enabled);
        if (!enabled && mOp.mObjLogger != null) {
            mOp.mObjLogger.camState.set(myVisionPortal.getCameraState().toString());
            ZeroOutLogger();
            mOp.mObjLogger.writeLine();
        }
    }
    private void ZeroOutLogger() {
        mOp.mObjLogger.numObjDet.set(0);
        mOp.mObjLogger.objX.set(0);
        mOp.mObjLogger.objY.set(0);
        mOp.mObjLogger.objConf.set(0);
        mOp.mObjLogger.numAprilDet.set(0);
        mOp.mObjLogger.aprilName.set("");
        mOp.mObjLogger.aprilRange.set(0);
        mOp.mObjLogger.aprilBearing.set(0);
        mOp.mObjLogger.aprilYaw.set(0);
    }
}
