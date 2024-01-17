package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem.FEET_PER_METER;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.commands.AlignmentScoringCommand;
import org.firstinspires.ftc.teamcode.subsystems.VisionPipelines.CameraStreamProcessor;
import org.firstinspires.ftc.teamcode.subsystems.VisionPipelines.RedDetectPipeline;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.TimeUnit;


public class AprilVision extends SubsystemBase {
    HardwareMap m_hardwareMap;
    Telemetry m_telemetry;
    RedDetectPipeline m_redProcessor;
    AprilTagProcessor m_tagProcessorFront;
    AprilTagProcessor m_tagProcessorBack;
    VisionPortal m_frontCamera;
    VisionPortal m_backCamera;
    public int width = 640;
    public int height = 480;

    private final int tagLeft = 10;
    private final int tagRight = 11;
    private final int tagMiddle = 12;
    private ArrayList<AprilTagDetection> detections;
    final CameraStreamProcessor processor = new CameraStreamProcessor();

    public AprilVision(HardwareMap hardwareMap, Telemetry telemetry){
        m_hardwareMap = hardwareMap;
        m_telemetry = telemetry;

       m_redProcessor = new RedDetectPipeline();
        int[] portalArray = VisionPortal.makeMultiPortalView(2, VisionPortal.MultiPortalLayout.HORIZONTAL);
        m_tagProcessorBack = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)

                .build();

        m_backCamera = new VisionPortal.Builder()
                .addProcessors(m_redProcessor, m_tagProcessorBack)
                .setCamera(m_hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(width, height))
                .enableLiveView(true)
                .setAutoStopLiveView(true)
                .setLiveViewContainerId(portalArray[0])
                .build();

        m_tagProcessorFront = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)

                .build();

        m_frontCamera = new VisionPortal.Builder()
                .addProcessors(processor, m_tagProcessorFront)
                .setCamera(m_hardwareMap.get(WebcamName.class, "Webcam 2"))
                .setCameraResolution(new Size(width, height))
                .enableLiveView(true)
                .setAutoStopLiveView(true)
                .setLiveViewContainerId(portalArray[1])
                .build();
            m_tagProcessorBack.setDecimation(2);
            m_tagProcessorFront.setDecimation(3);
        FtcDashboard.getInstance().startCameraStream(processor, 30);
      m_frontCamera.setProcessorEnabled(m_tagProcessorFront, true);
       m_backCamera.setProcessorEnabled(m_tagProcessorBack, false);
        setManualExposure(6, 250,m_backCamera);
        setManualExposure(6,250, m_frontCamera);
        m_telemetry.addData("redDetect", m_backCamera.getProcessorEnabled(m_redProcessor));
        m_telemetry.addData("aprilDetect", m_frontCamera.getProcessorEnabled(m_tagProcessorFront));
        m_telemetry.update();
    }


    public void disableCameras(){
        m_frontCamera.stopLiveView();
        m_frontCamera.stopStreaming();
        m_frontCamera.close();
        m_backCamera.stopLiveView();
        m_backCamera.stopStreaming();
        m_backCamera.close();
    }

    public void switchToAprilProcessorBack(){
        m_backCamera.setProcessorEnabled(m_redProcessor, false);
        m_backCamera.setProcessorEnabled(m_tagProcessorBack, true);
    }

    public void switchToAprilProcessorFront(){
        m_frontCamera.setProcessorEnabled(processor, false);
        m_frontCamera.setProcessorEnabled(m_tagProcessorFront, true);
    }



    public Command switchToAprilCommand(){return new InstantCommand(()-> this.switchToAprilProcessorBack());}



    public ArrayList<AprilTagDetection> getAprilTagDetectionsBack(){
        if(m_tagProcessorBack.getDetections() == null){return new ArrayList<AprilTagDetection>();}

        else{return m_tagProcessorBack.getDetections();}
    }

    public ArrayList<AprilTagDetection> getAprilTagDetectionsFront(){
        if(m_tagProcessorFront.getDetections() == null){return new ArrayList<AprilTagDetection>();}

        else{return m_tagProcessorFront.getDetections();}
    }


    public boolean seeingAprilTags(){return m_tagProcessorBack.getDetections() != null;}

    public boolean seeingAprilTagsFront(){return m_tagProcessorFront.getDetections() != null;}

    public AprilTagDetection getDesiredAprilTag(int DESIRED_TAG_ID) {   // Step through the list of detected tags and look for a matching tag
        List<AprilTagDetection> currentDetections = m_tagProcessorBack.getDetections();
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

    public AprilTagDetection getDesiredAprilTagFront(int DESIRED_TAG_ID) {   // Step through the list of detected tags and look for a matching tag
        List<AprilTagDetection> currentDetections = m_tagProcessorFront.getDetections();
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

    public double getFtcPoseX(int desiredTag){
            detections = getAprilTagDetectionsBack();
        for(AprilTagDetection detection : detections){
            if(detection.id == desiredTag){return detection.ftcPose.y;}
        }
        return 0;
    }


    public double getFtcPoseY(int desiredTag){
        detections = getAprilTagDetectionsBack();
        for(AprilTagDetection detection : detections){
            if(detection.id == desiredTag){return detection.ftcPose.x;}
        }
        return 0;
    }

    public double getFtcPoseZ(int desiredTag){
        detections = getAprilTagDetectionsBack();
        for(AprilTagDetection detection : detections){
            if(detection.id == desiredTag){return detection.ftcPose.yaw;}
        }
        return 0;
    }

    public double getPoseYaw(int desiredTag){
        detections = getAprilTagDetectionsBack();
        for(AprilTagDetection detection : detections){
            if(detection.id == desiredTag){return detection.ftcPose.yaw;}
        }
        return 0;
    }

    public double getPoseRange(int desiredTag){
        detections = getAprilTagDetectionsBack();
        for(AprilTagDetection detection : detections){
            if(detection.id == desiredTag){return detection.ftcPose.range;}
        }
        return 0;
    }

    public double getPoseBearing(int desiredTag){
        detections = getAprilTagDetectionsBack();
        for(AprilTagDetection detection : detections){
            if(detection.id == desiredTag){return detection.ftcPose.bearing;}
        }
        return 0;
    }

    public double getFtcPoseXF(int desiredTag){
        detections = getAprilTagDetectionsFront();
        for(AprilTagDetection detection : detections){
            if(detection.id == desiredTag){return detection.ftcPose.y;}
        }
        return 0;
    }


    public double getFtcPoseYF(int desiredTag){
        detections = getAprilTagDetectionsFront();
        for(AprilTagDetection detection : detections){
            if(detection.id == desiredTag){return detection.ftcPose.x;}
        }
        return 0;
    }

    public double getFtcPoseZF(int desiredTag){
        detections = getAprilTagDetectionsFront();
        for(AprilTagDetection detection : detections){
            if(detection.id == desiredTag){return detection.ftcPose.yaw;}
        }
        return 0;
    }

    public double getPoseYawF(int desiredTag){
        detections = getAprilTagDetectionsFront();
        for(AprilTagDetection detection : detections){
            if(detection.id == desiredTag){return detection.ftcPose.yaw;}
        }
        return 0;
    }

    public double getPoseRangeF(int desiredTag){
        detections = getAprilTagDetectionsFront();
        for(AprilTagDetection detection : detections){
            if(detection.id == desiredTag){return detection.ftcPose.range;}
        }
        return 0;
    }

    public double getPoseBearingF(int desiredTag){
        detections = getAprilTagDetectionsFront();
        for(AprilTagDetection detection : detections){
            if(detection.id == desiredTag){return detection.ftcPose.bearing;}
        }
        return 0;
    }






//    public void getSpecificTag(int pos){
//        m_tagProcessor.getDetections().get(i)
//        for()
//    }

        private void setManualExposure(int exposureMS, int gain, VisionPortal m_visionPortal) {
            // Wait for the camera to be open, then use the controls

            if (m_visionPortal == null) {
                return;
            }
            if (m_visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
                m_telemetry.addData("Camera", "Waiting");
                m_telemetry.update();
                while ((m_visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                    new WaitCommand(30);
                }
                m_telemetry.addData("Camera", "Ready");
                m_telemetry.update();
            }
            // Set camera controls unless we are stopping.

                ExposureControl exposureControl = m_visionPortal.getCameraControl(ExposureControl.class);
                if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                    exposureControl.setMode(ExposureControl.Mode.Manual);

                }
                exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);

                GainControl gainControl = m_visionPortal.getCameraControl(GainControl.class);
                gainControl.setGain(gain);


        }


    public Command changeSpot(int spot){
        return new InstantCommand(()-> AlignmentScoringCommand.m_position = spot);
    }

    public int getLocation(){return m_redProcessor.getLocation();}

    @Override
    public void periodic(){
        m_telemetry.addData("redDetect", m_backCamera.getProcessorEnabled(m_redProcessor));
        m_telemetry.addData("aprilDetect", m_backCamera.getProcessorEnabled(m_tagProcessorBack));


//        m_telemetry.addData("redDetectFront", m_backCamera.getProcessorEnabled(m_redProcessor));
        m_telemetry.addData("aprilDetectFront", m_frontCamera.getProcessorEnabled(m_tagProcessorFront));


        m_telemetry.addData("location", m_redProcessor.getLocation());
        ArrayList<AprilTagDetection> tags = getAprilTagDetectionsFront();
        if(m_tagProcessorFront.getDetections().size() > 0){
            for(AprilTagDetection tag : tags) {
                if(tag!= null) {
                    m_telemetry.addData("X", tag.ftcPose.x);
                    m_telemetry.addData("Y", tag.ftcPose.y);
                    m_telemetry.addData("Z", tag.ftcPose.z);
                    m_telemetry.addData("yaw", getPoseYawF(7));
                    m_telemetry.addData("pitch", tag.ftcPose.pitch);
                    m_telemetry.addData("roll", tag.ftcPose.roll);
                    m_telemetry.addData("range", getPoseRangeF(7));
                    m_telemetry.addData("Bearing", getPoseBearingF(7));

                }
            }
        }

    }
}
