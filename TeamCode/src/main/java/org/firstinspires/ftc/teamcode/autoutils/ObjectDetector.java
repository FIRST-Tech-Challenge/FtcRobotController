package org.firstinspires.ftc.teamcode.autoutils;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.util.Other.ArrayTypeValue;
import org.firstinspires.ftc.teamcode.util.Other.DynamicTypeValue;
import org.firstinspires.ftc.teamcode.util.RobotHardwareInitializer;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Objects;

import static java.util.Locale.ENGLISH;
import static org.firstinspires.ftc.teamcode.util.RobotHardwareInitializer.Other.WEBCAM;
import static org.firstinspires.ftc.teamcode.util.RobotHardwareInitializer.Cameras;

import org.firstinspires.ftc.teamcode.util.FTCDashboardPackets;

public class ObjectDetector {
    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    /**
     * The variable to store our instance of the AprilTag processor.
     */
    private AprilTagProcessor aprilTag;
    private final WebcamName Camera1, Camera2;

    // TFOD_MODEL_ASSET points to a model file stored in the project Asset location,
    // this is only used for Android Studio when using models in Assets.
    private static final String TFOD_MODEL_ASSET = "cube.tflite";
    // TFOD_MODEL_FILE points to a model file stored onboard the Robot Controller's storage,
    // this is used when uploading models directly to the RC using the model upload interface.
    private static final String TFOD_MODEL_FILE = "/sdcard/FIRST/tflitemodels/myCustomModel.tflite";
    // Define the labels recognized in the model for TFOD (must be in training order!)
    private static final String[] LABELS = {
            "Cube",
            "Not the Cube"
    };

    public static final FTCDashboardPackets dbp = new FTCDashboardPackets("OBJ_DETECTOR");

    /**
     * The variable to store our instance of the TensorFlow Object Detection processor.
     */
    private TfodProcessor tfod;

    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;

    private final HashMap<RobotHardwareInitializer.Other, DynamicTypeValue> OTHER;

    public ObjectDetector(final HashMap<RobotHardwareInitializer.Other, DynamicTypeValue> other) {
        OTHER = other;
        dbp.createNewTelePacket();

        Camera1 =
                (WebcamName) ((ArrayTypeValue<?>) Objects.requireNonNull(OTHER.get(WEBCAM))).get(0);

        Camera2 =
                (WebcamName) ((ArrayTypeValue<?>) Objects.requireNonNull(OTHER.get(WEBCAM))).get(1);

        initTfod();
    }

    public void stopCamera() {
        visionPortal.close();
    }

    public void setCamera(Cameras camera) {
        if (visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING) {
            if (camera == Cameras.CAM1) {
                visionPortal.setActiveCamera(Camera1);
            } else if (camera == Cameras.CAM2) {
                visionPortal.setActiveCamera(Camera2);
            }
        }
    }

    /**
     * Initialize the TensorFlow Object Detection processor.
     */
    private void initTfod() {

        // Create the TensorFlow processor by using a builder.
        tfod = new TfodProcessor.Builder()

                // With the following lines commented out, the default TfodProcessor Builder
                // will load the default model for the season. To define a custom model to load,
                // choose one of the following:
                //   Use setModelAssetName() if the custom TF Model is built in as an asset (AS only).
                //   Use setModelFileName() if you have downloaded a custom team model to the Robot Controller.
                .setModelAssetName(TFOD_MODEL_ASSET)
                //.setModelFileName(TFOD_MODEL_FILE)

                // The following default settings are available to un-comment and edit as needed to
                // set parameters for custom models.
                .setModelLabels(LABELS)
                .setIsModelTensorFlow2(true)
                .setIsModelQuantized(true)
                .setModelInputSize(300)
                .setModelAspectRatio(16.0 / 9.0)

                .build();

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        if (USE_WEBCAM) {
            builder.setCamera((CameraName) Camera1);
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        // Choose a camera resolution. Not all cameras support all resolutions.
        //builder.setCameraResolution(new Size(640, 480));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        //builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(tfod);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Set confidence threshold for TFOD recognitions, at any time.
        //tfod.setMinResultConfidence(0.75f);

        // Disable or re-enable the TFOD processor at any time.
        //visionPortal.setProcessorEnabled(tfod, true);

    }   // end method initTfod()

    public boolean isObjectRecognized(final String label) {
        final List<Recognition> RECOGNITIONS = telemetryTfod();

        for (Recognition recognition : RECOGNITIONS) {
            if (Objects.equals(recognition.getLabel(), label)) return true;
        }

        return false;
    }

    /**
     * Add telemetry about TensorFlow Object Detection (TFOD) recognitions.
     */
    private List<Recognition> telemetryTfod() {

        List<Recognition> currentRecognitions = tfod.getRecognitions();
        dbp.info(String.format(ENGLISH, "%d Objects Detected", currentRecognitions.size()));

        // Step through the list of recognitions and display info for each one.
        for (Recognition recognition : currentRecognitions) {
            double x = (recognition.getLeft() + recognition.getRight()) / 2 ;
            double y = (recognition.getTop()  + recognition.getBottom()) / 2 ;

            dbp.put("Image", String.format(ENGLISH, "%s (%f.0 %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100));
            dbp.put("- Position", String.format(ENGLISH, "%f / %f", x, y));
            dbp.put("- Size", String.format(ENGLISH, "%f x %f", recognition.getWidth(), recognition.getHeight()));
            dbp.send(false);
        }

        return currentRecognitions;
    }

    /**
     * Initialize the AprilTag processor.
     */
    private void initAprilTag() {

        // Create the AprilTag processor.
        aprilTag = new AprilTagProcessor.Builder()

                // The following default settings are available to un-comment and edit as needed.
                //.setDrawAxes(false)
                //.setDrawCubeProjection(false)
                //.setDrawTagOutline(true)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                //.setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)

                // == CAMERA CALIBRATION ==
                // If you do not manually specify calibration parameters, the SDK will attempt
                // to load a predefined calibration for your camera.
                //.setLensIntrinsics(578.272, 578.272, 402.145, 221.506)
                // ... these parameters are fx, fy, cx, cy.

                .build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second (default)
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second (default)
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        //aprilTag.setDecimation(3);

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        if (USE_WEBCAM) {
            builder.setCamera((CameraName) Objects.requireNonNull(OTHER.get(WEBCAM)).getValue());
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        // Choose a camera resolution. Not all cameras support all resolutions.
        //builder.setCameraResolution(new Size(640, 480));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        //builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(aprilTag);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Disable or re-enable the aprilTag processor at any time.
        //visionPortal.setProcessorEnabled(aprilTag, true);

    }

    public enum AprilTags {
        BLUE_ALLIANCE_LEFT,
        BLUE_ALLIANCE_CENTER,
        BLUE_ALLIANCE_RIGHT,
        RED_ALLIANCE_LEFT,
        RED_ALLIANCE_CENTER,
        RED_ALLIANCE_RIGHT
    }

    public static AprilTags aprilTagIdToEnumType(final int id) {
        switch (id - 1) {
            case 0:
                return AprilTags.BLUE_ALLIANCE_LEFT;
            case 1:
                return AprilTags.BLUE_ALLIANCE_CENTER;
            case 2:
                return AprilTags.BLUE_ALLIANCE_RIGHT;
            case 3:
                return AprilTags.RED_ALLIANCE_LEFT;
            case 4:
                return AprilTags.RED_ALLIANCE_CENTER;
            case 5:
                return AprilTags.RED_ALLIANCE_RIGHT;
        }

        return null;
    }

    public static List<AprilTags> getDetectedAprilTagIds(final List<AprilTagDetection> detections) {
        List<AprilTags> ids = new ArrayList<>();
        for (AprilTagDetection detection : detections) {
            ids.add(aprilTagIdToEnumType(detection.id - 1));
        }
        return ids;
    }

    public boolean isAprilTagRecognized(final AprilTags tag) {
        List<AprilTagDetection> currentDetections = telemetryAprilTag();

        return getDetectedAprilTagIds(currentDetections).contains(tag);
    }


    /**
     * Add telemetry about AprilTag detections.
     */
    private List<AprilTagDetection> telemetryAprilTag() {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        dbp.info(String.format(ENGLISH, "%d AprilTags Detected", currentDetections.size()));

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                dbp.info(String.format(ENGLISH, "\n==== (ID %d) %s",
                        detection.id,
                        detection.metadata.name)
                );
                dbp.info(String.format(ENGLISH, "XYZ %6.1f %6.1f %6.1f  (inch)",
                        detection.ftcPose.x,
                        detection.ftcPose.y,
                        detection.ftcPose.z)
                );
                dbp.info(String.format(ENGLISH, "PRY %6.1f %6.1f %6.1f  (deg)",
                        detection.ftcPose.pitch,
                        detection.ftcPose.roll,
                        detection.ftcPose.yaw)
                );
                dbp.info(String.format(ENGLISH, "RBE %6.1f %6.1f %6.1f  (inch, deg, deg)",
                        detection.ftcPose.range,
                        detection.ftcPose.bearing,
                        detection.ftcPose.elevation)
                );
            } else {
                dbp.info(String.format(ENGLISH, "\n==== (ID %d) Unknown", detection.id));
                dbp.info(String.format(ENGLISH, "Center %6.0f %6.0f   (pixels)",
                        detection.center.x, detection.center.y));
            }
            dbp.send(false);
        }   // end for() loop

        // Add "key" information to telemetry
        dbp.info("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        dbp.info("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        dbp.info("RBE = Range, Bearing & Elevation");

        dbp.send(false);

        return currentDetections;
    }
}
