package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagSingleDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
@Config
public class CameraTrackingSubsystems {
    private WebcamName webcam1;
    private Telemetry telemetry;
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    public CameraTrackingSubsystems(HardwareMap hardwareMap, Telemetry telemetry){
        this.telemetry = telemetry;
        webcam1 = hardwareMap.get(WebcamName.class, "Webcam 1");

        // 1. Build the AprilTag Processor
        aprilTag = new AprilTagProcessor.Builder()
                .build();

        // 2. Build the VisionPortal linking your webcam and processor
        visionPortal = new VisionPortal.Builder()
                .setCamera(webcam1)
                .addProcessor(aprilTag)
                .build();
    }

    public double autoAim(){
        FtcDashboard.getInstance().startCameraStream(visionPortal,0);
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();

        for (AprilTagDetection detection : currentDetections) {


            // Check if it's a Cluster Detection
             if (detection instanceof AprilTagClusterDetection) {
                AprilTagClusterDetection clusterDet = (AprilTagClusterDetection) detection;

                // Clusters don't have a single ID, but you can target them based on metadata name or presence
                if (clusterDet.metadata != null) {
                    telemetry.addData("Target Cluster Found", clusterDet.metadata.name);
                    telemetry.addData("Percent Found", "%d%%", clusterDet.percentClusterFound);

                    if (clusterDet.ftcPose != null) {
                        telemetry.addData("Yaw (Error)", "%5.2f degrees", clusterDet.ftcPose.yaw);
                        return clusterDet.ftcPose.yaw;

                    }
                }
            }
        }

        telemetry.addData("Target", "Not Found");
        return 0.0;
    }

    public void closePortal() {
        if (visionPortal != null) {
            visionPortal.close();
        }
    }

    public boolean seeApirltag() {
        FtcDashboard.getInstance().startCameraStream(visionPortal,0);
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            if (detection instanceof AprilTagClusterDetection) {
                return true;
            }

        }
        return false;
    }
}
