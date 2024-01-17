package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Size;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.robotContainer.Constants;
import org.firstinspires.ftc.teamcode.subsystems.VisionPipelines.RedDetectPipeline;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.List;

public class AprilVision2 extends SubsystemBase {

    private VisionPortal m_visionPortal;
    private AprilTagProcessor m_aprilTagProcessor;
    RedDetectPipeline m_redProcessor;
    private HardwareMap m_hardwareMap;
    private Telemetry m_telemetry;
    private WebcamName webcam1;
    private WebcamName webcam2;
    ArrayList<AprilTagDetection> detections;

    public AprilVision2(HardwareMap hardwareMap, Telemetry telemetry) {
        m_hardwareMap = hardwareMap;
        m_telemetry = telemetry;
        // Create the AprilTag processor by using a builder.
        m_aprilTagProcessor = new AprilTagProcessor.Builder().build();
        m_redProcessor = new RedDetectPipeline();


        webcam1 = hardwareMap.get(WebcamName.class, "Webcam 1");


        // Create the vision portal by using a builder.
        m_visionPortal = new VisionPortal.Builder()
                .setCamera(webcam1)
                .addProcessors(m_redProcessor, m_aprilTagProcessor)
                .build();
        m_visionPortal.setProcessorEnabled(m_aprilTagProcessor, false);


    }

    public void switchToApril(){
        m_visionPortal.setProcessorEnabled(m_redProcessor, false);
        m_visionPortal.setProcessorEnabled(m_aprilTagProcessor, true);
    }
    public ArrayList<AprilTagDetection> getAprilTagDetections() {
        if (m_aprilTagProcessor.getDetections() == null) {
            return new ArrayList<AprilTagDetection>();
        } else {
            return m_aprilTagProcessor.getDetections();
        }
    }


    public boolean seeingAprilTags() {
        return m_aprilTagProcessor.getDetections() != null;
    }


    public AprilTagDetection getDesiredAprilTag(int DESIRED_TAG_ID) {   // Step through the list of detected tags and look for a matching tag
        List<AprilTagDetection> currentDetections = m_aprilTagProcessor.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            // Look to see if we have size info on this tag.
            if (detection.metadata != null) {
                //  Check to see if we want to track towards this tag.
                if ((DESIRED_TAG_ID < 0) || (detection.id == DESIRED_TAG_ID)) {
                    // Yes, we want to use this tag.
                    return detection;
                    // don't look any further.
                } else {
                    // This tag is in the library, but we do not want to track it right now.
                    m_telemetry.addData("Skipping", "Tag ID %d is not desired", detection.id);
                }
            } else {
                // This tag is NOT in the library, so we don't have enough information to track to it.
                m_telemetry.addData("Unknown", "Tag ID %d is not in TagLibrary", detection.id);
            }
        }
        return null;
    }
//
//    public double getFtcPoseX(int desiredTag) {
//        detections = getAprilTagDetections();
//        for (AprilTagDetection detection : detections) {
//            if (detection.id == desiredTag) {
//                return detection.ftcPose.y;
//            }
//        }
//        return 0;
//    }
//
//
//    public double getFtcPoseY(int desiredTag) {
//        detections = getAprilTagDetections();
//        for (AprilTagDetection detection : detections) {
//            if (detection.id == desiredTag) {
//                return detection.ftcPose.x;
//            }
//        }
//        return 0;
//    }
//
//    public double getFtcPoseZ(int desiredTag) {
//        detections = getAprilTagDetections();
//        for (AprilTagDetection detection : detections) {
//            if (detection.id == desiredTag) {
//                return detection.ftcPose.yaw;
//            }
//        }
//        return 0;
//    }
//
//    public double getPoseYaw(int desiredTag) {
//        detections = getAprilTagDetections();
//        for (AprilTagDetection detection : detections) {
//            if (detection.id == desiredTag) {
//                return detection.ftcPose.yaw;
//            }
//        }
//        return 0;
//    }
//
//    public double getPoseRange(int desiredTag) {
//        detections = getAprilTagDetections();
//        for (AprilTagDetection detection : detections) {
//            if (detection.id == desiredTag) {
//                return detection.ftcPose.range;
//            }
//        }
//        return 0;
//    }
//
//    public double getPoseBearing(int desiredTag) {
//        detections = getAprilTagDetections();
//        for (AprilTagDetection detection : detections) {
//            if (detection.id == desiredTag) {
//                return detection.ftcPose.bearing;
//            }
//        }
//        return 0;
//    }



    public void disableCameras() {
        m_visionPortal.close();

    }

    public void telemetryCameraSwitching() {

        if (m_visionPortal.getActiveCamera().equals(webcam1)) {
            m_telemetry.addData("activeCamera", "Webcam 1");
            m_telemetry.addData("Press RightBumper", "to switch to Webcam 2");
        } else {
            m_telemetry.addData("activeCamera", "Webcam 2");
            m_telemetry.addData("Press LeftBumper", "to switch to Webcam 1");
        }

    }
        public void setMode(String mode) {

        if (mode == "BLUE") {
            m_redProcessor.H_start = Constants.H_START_BLUE;
            m_redProcessor.H_end = Constants.H_END_BLUE;
        } else if (mode == "RED") {
            m_redProcessor.H_start = Constants.H_START_RED;
            m_redProcessor.H_end = Constants.H_END_RED;

        }
    }

//    public void telemetryAprilTag() {
//
//        List<AprilTagDetection> currentDetections = m_aprilTagProcessor.getDetections();
//        m_telemetry.addData("# AprilTags Detected", currentDetections.size());
//
//        // Step through the list of detections and display info for each one.
//        for (AprilTagDetection detection : currentDetections) {
//            if (detection.metadata != null) {
//                m_telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
//                m_telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
//                m_telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, getPoseYaw(7)));
//                m_telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, getPoseBearing(7), detection.ftcPose.elevation));
//            } else {
//                m_telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
//                m_telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
//            }
//        }   // end for() loop
//
//        // Add "key" information to telemetry
//        m_telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
//        m_telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
//        m_telemetry.addLine("RBE = Range, Bearing & Elevation");
//
//    }

    public int getLocation(){

       return m_redProcessor.getLocation();
        }

    @Override
    public void periodic() {


//            List<AprilTagDetection> currentDetections = m_aprilTagProcessor.getDetections();
//        m_telemetry.addData("# AprilTags Detected", currentDetections.size());
//
//            // Step through the list of detections and display info for each one.
//            for (AprilTagDetection detection : currentDetections) {
//                if (detection.metadata != null) {
//                    m_telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
//                    m_telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
//                    m_telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
//                    m_telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
//                } else {
//                    m_telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
//                    m_telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
//                }
//            }   // end for() loop
//
//            // Add "key" information to telemetry
//        m_telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
//        m_telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
//            m_telemetry.addLine("RBE = Range, Bearing & Elevation");


//        m_telemetry.addData("redDetect", m_backCamera.getProcessorEnabled(m_redProcessor));
//        m_telemetry.addData("aprilDetect", m_backCamera.getProcessorEnabled(m_tagProcessorBack));
//
//
////        m_telemetry.addData("redDetectFront", m_backCamera.getProcessorEnabled(m_redProcessor));
//        m_telemetry.addData("aprilDetectFront", m_frontCamera.getProcessorEnabled(m_tagProcessorFront));
//
//
            m_telemetry.addData("location", m_redProcessor.getLocation());

        m_telemetry.update();
    }
}

