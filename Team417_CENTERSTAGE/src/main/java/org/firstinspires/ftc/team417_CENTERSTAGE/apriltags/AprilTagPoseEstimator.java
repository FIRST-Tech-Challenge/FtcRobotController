package org.firstinspires.ftc.team417_CENTERSTAGE.apriltags;

import static org.firstinspires.ftc.team417_CENTERSTAGE.roadrunner.MecanumDrive.isDevBot;

import android.util.Size;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.team417_CENTERSTAGE.baseprograms.BaseOpMode;
import org.firstinspires.ftc.team417_CENTERSTAGE.utilityclasses.AprilTagInfo;
import org.firstinspires.ftc.team417_CENTERSTAGE.utilityclasses.InfoWithDetection;
import org.firstinspires.ftc.team417_CENTERSTAGE.utilityclasses.Pose;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;

@Config
public class AprilTagPoseEstimator {
    public static double CAMERA_LATENCY = 640; // latency between april tag shown and detection in ms

    public static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    public HardwareMap myHardwareMap;   // gain access to camera in hardwareMap

    public Telemetry telemetry;   // gain access to telemetry

    /**
     * The variable to store our instance of the AprilTag processor.
     */
    public AprilTagProcessor aprilTag;

    /**
     * The variable to store our instance of the status light.
     */
    public DigitalChannel statusLight;

    /**
     * The variable to store our instance of the vision portal.
     */
    public VisionPortal visionPortal;

    // For this concept: supposed location of the robot
    Pose robotPoseEstimate = new Pose(0, 0, 0);

    public AprilTagPoseEstimator(HardwareMap hardwareMap, Telemetry telemetry) {
        this.myHardwareMap = hardwareMap;
        this.telemetry = telemetry;
        init();
    }

    public AprilTagPoseEstimator(HardwareMap hardwareMap) {
        new AprilTagPoseEstimator(hardwareMap, null);
    }

    /**
     * Initialize the AprilTag processor.
     */
    public void init() {
        // Create the AprilTag processor.
        aprilTag = new AprilTagProcessor.Builder()

                // The following default settings are available to un-comment and edit as needed.
                //.setDrawAxes(false)
                //.setDrawCubeProjection(false)
                //.setDrawTagOutline(true)
                //.setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                //.setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
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
        // Changed from 3 (default) to 1
        aprilTag.setDecimation(1);

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        if (USE_WEBCAM) {
            builder.setCamera(myHardwareMap.get(WebcamName.class, "webcam"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        // Sets the status light for latency testing, etc.
        if (isDevBot) {
            statusLight = myHardwareMap.get(DigitalChannel.class, "green");
            statusLight.setMode(DigitalChannel.Mode.OUTPUT);
        }

        if (statusLight != null) {
            statusLight.setState(true);
        }

        // Choose a camera resolution. Not all cameras support all resolutions.
        // Used max resolution
        builder.setCameraResolution(new Size(1920, 1080));

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

    }   // end method initAprilTag()

    // Calculates the pose estimate based on a detection of an april tag
    //     and the stored information of said april tag (position etc.)
    // Also telemeters relevant info
    // Note that this is relative to the center of the camera
    public Pose calculatePoseEstimate(AprilTagDetection detection, AprilTagInfo aprilTagInfo) {
        // See November notebook 11/20/2023 for more info on the math used here

        // Declaring variables
        double d, beta, gamma, relativeX, relativeY, absoluteX, absoluteY, absoluteTheta;

        if (isDevBot) {
            // d - absolute distance from April-tag to robot
            d = Math.hypot(detection.ftcPose.x + Constants.DEVBOT_CAMERA_TO_CENTER_X, detection.ftcPose.y + Constants.DEVBOT_CAMERA_TO_CENTER_Y);

            // gamma - angle of center of camera direction to april tag direction
            gamma = Math.atan2(detection.ftcPose.x + Constants.DEVBOT_CAMERA_TO_CENTER_X, detection.ftcPose.y + Constants.DEVBOT_CAMERA_TO_CENTER_Y);

            // beta - yaw of robot relative to the tag
            beta = gamma + Math.toRadians(detection.ftcPose.yaw + Constants.DEVBOT_CAMERA_TO_CENTER_ROT); //(or gamma - detection.ftcPose.yaw + + Constants.DEVBOT_CAMERA_TO_CENTER_ROT) if that doesn't work)
        } else {
            // d - absolute distance from April-tag to robot
            d = Math.hypot(detection.ftcPose.x + Constants.COMPETITION_BOT_FRONT_CAMERA_TO_CENTER_X, detection.ftcPose.y + Constants.COMPETITION_BOT_FRONT_CAMERA_TO_CENTER_Y);

            // gamma - angle of center of camera direction to april tag direction
            gamma = Math.atan2(detection.ftcPose.x + Constants.COMPETITION_BOT_FRONT_CAMERA_TO_CENTER_X, detection.ftcPose.y + Constants.COMPETITION_BOT_FRONT_CAMERA_TO_CENTER_Y);

            // beta - yaw of robot relative to the tag
            beta = gamma + Math.toRadians(detection.ftcPose.yaw + Constants.COMPETITION_BOT_FRONT_CAMERA_TO_CENTER_ROT); //(or gamma - detection.ftcPose.yaw + + Constants.COMPETITION_BOT_FRONT_CAMERA_TO_CENTER_ROT) if that doesn't work)
        }

        // relativeX - x of robot without compensating for yaw
        relativeX = d * Math.cos(beta) + aprilTagInfo.x;

        // relativeY - y of robot without compensating for yaw
        relativeY = -d * Math.sin(beta) + aprilTagInfo.y;

        // absoluteX - x of robot
        absoluteX = relativeX * Math.cos(Math.toRadians(aprilTagInfo.yaw)) - relativeY * Math.sin(Math.toRadians(aprilTagInfo.yaw));

        // absoluteY - y of robot
        absoluteY = relativeX * Math.sin(Math.toRadians(aprilTagInfo.yaw)) + relativeY * Math.cos(Math.toRadians(aprilTagInfo.yaw));

        // absoluteTheta - yaw of robot
        absoluteTheta = Math.toRadians(aprilTagInfo.yaw) - Math.toRadians(detection.ftcPose.yaw) + Math.PI;

        return new Pose(absoluteX, absoluteY, absoluteTheta);
    }

    // Many camera frames contain two or more AprilTag detections.
    // This method chooses the best detection to go off of for a pose estimate.
    public InfoWithDetection chooseBestAprilTag(ArrayList<InfoWithDetection> iwdList) {
        // If list is empty, return null
        if (iwdList.size() < 1) {
            return null;
        }

        // Remove iwds (InfoWithDetection objects) that are more than two tiles (48 inches) away
        int iwdListSize = iwdList.size();
        InfoWithDetection currentIwd;
        for (int i = 0; i < iwdListSize; i++) {
            currentIwd = iwdList.get(i);
            if (Math.hypot(currentIwd.detection.ftcPose.x, currentIwd.detection.ftcPose.y) > Constants.MAX_DETECTION_DISTANCE) {
                iwdList.remove(i);
                i--;
            }
            iwdListSize = iwdList.size();
        }

        // NOW if list is empty, return null
        if (iwdList.size() < 1) {
            return null;
        }

        // Find the largest size of April Tag
        currentIwd = iwdList.get(0);
        double largestSize = currentIwd.info.sideLength;
        for (int i = 1; i < iwdList.size(); i++) {
            currentIwd = iwdList.get(i);
            if (currentIwd.info.sideLength > largestSize) {
                largestSize = currentIwd.info.sideLength;
            }
        }

        // Remove iwds that are not the largest
        for (int i = 0; i < iwdListSize; i++) {
            currentIwd = iwdList.get(i);
            if (!BaseOpMode.isEpsilonEquals(currentIwd.info.sideLength, largestSize) && currentIwd.info.sideLength < largestSize) {
                iwdList.remove(i);
                i--;
            }
            iwdListSize = iwdList.size();
        }

        // Choose the april tag that has the least distance
        currentIwd = iwdList.get(0);
        double leastDistance = Math.hypot(currentIwd.detection.ftcPose.x, currentIwd.detection.ftcPose.y);
        int leastDistanceIndex = 0;
        for (int i = 1; i < iwdList.size(); i++) {
            currentIwd = iwdList.get(i);
            if (Math.hypot(currentIwd.detection.ftcPose.x, currentIwd.detection.ftcPose.y) < leastDistance) {
                leastDistance = Math.hypot(currentIwd.detection.ftcPose.x, currentIwd.detection.ftcPose.y);
                leastDistanceIndex = i;
            }
        }

        return iwdList.get(leastDistanceIndex);
    }

    ArrayList<InfoWithDetection> knownAprilTagsDetected = new ArrayList<>();

    /**
     * Produce a pose estimate from current frames and update it
     */
    public void updatePoseEstimate() {
        ArrayList<AprilTagDetection> currentDetections = aprilTag.getDetections();

        // Iterates through detections and finds any the the robot "knows"
        // Then it replaces the pose estimate with pose estimate from that april tag
        knownAprilTagsDetected.clear();
        for (AprilTagDetection detection : currentDetections) {
            AprilTagInfo aprilTagInfo = AprilTagInfoDump.findTagWithId(detection.id);
            if (aprilTagInfo != null) {
                knownAprilTagsDetected.add(new InfoWithDetection(aprilTagInfo, detection));
            }
        }

        boolean detecting;
        InfoWithDetection best = chooseBestAprilTag(knownAprilTagsDetected);
        if (best != null) {
            robotPoseEstimate = calculatePoseEstimate(best.detection, best.info);
            detecting = true;
        } else {
            robotPoseEstimate = null;
            detecting = false;
        }

        // Turn the status light on when it detects an april tag (yes, setState(boolean) is backwards)
        if (statusLight != null) {
            statusLight.setState(!detecting);
        }

        // Telemeters the current pose estimate
        if (telemetry != null && robotPoseEstimate != null) {
            telemetry.addLine(String.format("Robot XYθ %6.1f %6.1f %6.1f  (inch) (degrees)", robotPoseEstimate.x, robotPoseEstimate.y, Math.toDegrees(robotPoseEstimate.theta)));

            // Telemeters the pose info to FTC dashboard so that it draws the robot pose
            // Remove before competition, could cause lags
            //c.setStroke("#3F51B5");
            //MecanumDrive.drawRobot(c, new Pose2d(robotPoseEstimate.x, robotPoseEstimate.y, robotPoseEstimate.theta));

        /*
        myOpMode.telemetry.addData("\n# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                myOpMode.telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                myOpMode.telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x + + Constants.DEVBOT_CAMERA_TO_CENTER_X, detection.ftcPose.y + + Constants.DEVBOT_CAMERA_TO_CENTER_Y, detection.ftcPose.z));
                myOpMode.telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw + + Constants.DEVBOT_CAMERA_TO_CENTER_ROT));
                myOpMode.telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
            } else {
                myOpMode.telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                myOpMode.telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }   // end for() loop

        // Add "key" information to telemetry
        myOpMode.telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        myOpMode.telemetry.addLine("PRY = Pitch, Roll & yaw) (XYZ Rotation)");
        myOpMode.telemetry.addLine("RBE = Range, Bearing & Elevation");
        */
        }
    } // end method telemetryAprilTag()

    public Pose2d estimatePose() {
        if (robotPoseEstimate != null) {
            return new Pose2d(robotPoseEstimate.x, robotPoseEstimate.y, robotPoseEstimate.theta);
        } else {
            return null;
        }
    }
}
