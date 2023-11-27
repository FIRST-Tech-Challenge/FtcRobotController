package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.util.LogFiles.log;

import android.util.Log;

import static org.firstinspires.ftc.teamcode.configPID.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

@Autonomous
public class apriltagtest extends LinearOpMode {
    private static final int DESIRED_TAG_ID = 1;     // Choose the tag you want to approach or set to -1 for ANY tag.
    boolean targetFound = false;    // Set to true when an AprilTag target is detected
    private AprilTagDetection desiredTag = null;     // Used to hold the data for a detected AprilTag
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    Pid_Controller_Adevarat pid = new Pid_Controller_Adevarat(pcam, icam, dcam);
    ChestiiDeAutonom c = new ChestiiDeAutonom();
    double pidRes;

    public void initAprilTag() {
        // Create the AprilTag processor by using a builder.
        aprilTag = new AprilTagProcessor.Builder().build();

        aprilTag.setDecimation(2);

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();
    }

    public void setManualExposure(int exposureMS, int gain) {
        // Wait for the camera to be open, then use the controls

        if (visionPortal == null) {
            return;
        }

        // Make sure camera is streaming before we try to set the exposure controls
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();
            while (!isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }

        // Set camera controls unless we are stopping.
        if (!isStopRequested()) {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                sleep(50);
            }
            exposureControl.setExposure((long) exposureMS, TimeUnit.MILLISECONDS);
            sleep(20);
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            sleep(20);
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        pid.enable();
        initAprilTag();
        c.initSasiu(hardwareMap);
        setManualExposure(6, 250);
        waitForStart();
        // Step through the list of detected tags and look for a matching tag
        while (!isStopRequested()) {
            pid.setPID(pcam, icam, dcam);
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            for (AprilTagDetection detection : currentDetections) {
                // Look to see if we have size info on this tag.
                if (detection.metadata != null) {
                    //  Check to see if we want to track towards this tag.
                    if (detection.id % 3 == 0) {
                        targetFound = true;
                        telemetry.addData("pid error:", pid.getError());
                        telemetry.addData("pid result", pidRes);
                        telemetry.addData("April tag detection id:", detection.id);
                        telemetry.addData("April tag center x:", pid.getSetpoint());
                        telemetry.addData("Yaw:", detection.ftcPose.yaw);
                        telemetry.addData("Range:", detection.ftcPose.range);
                        telemetry.addData("Bearing:", detection.ftcPose.bearing);
                        telemetry.update();
                        break;  // don't look any further.
                    } else {
                        // This tag is in the library, but we do not want to track it right now.
                        telemetry.addData("Skipping", "Tag ID %d is not desired", detection.id);
                    }
                }
            }
        }
    }
}