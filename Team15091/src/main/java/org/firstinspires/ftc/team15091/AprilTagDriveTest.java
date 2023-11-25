package org.firstinspires.ftc.team15091;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

@Autonomous(name = "AprilTagDriveTest", preselectTeleOp = "Gamepad")
public class AprilTagDriveTest extends AutonomousBase{
    // arm servo: 0 is out, 0.8 is in
    // bowl servo: 1 is out, 0 is in

    @Override
    public void runOpMode() throws InterruptedException {
        setupAndWait();
        List<AprilTagDetection> aprilTagDetections;
        while(opModeIsActive()) {
            aprilTagDetections = aprilTagDetector.scanForAprilTags();
            if (!aprilTagDetections.isEmpty()) {
                positionToAprilTag(1);
                break;
            }
        }
        }

    /**
     * Drives to a detected April Tag.
     * @param focusId The ID of the tag to drive to, if detected by the camera.
     */
    private void positionToAprilTag (int focusId) {
        List<AprilTagDetection> aprilTagDetections = aprilTagDetector.scanForAprilTags();
        AprilTagDetection focusedAprilTag = aprilTagDetector.scanForAprilTagById(focusId);
        if (focusedAprilTag != null) {
            AprilTagDetection tempAprilTag;
            double currentBearing = focusedAprilTag.ftcPose.bearing;
            double currentRange = focusedAprilTag.ftcPose.range;
            double currentYaw = focusedAprilTag.ftcPose.yaw;
            while (Math.abs(currentBearing) > 2) { // while the robot is not facing the correct bearing
                double adjustAngle = currentBearing;
                if (adjustAngle > 0) { // angle is positive (to the left)
                    adjustAngle = Math.min(5, currentBearing);
                }
                else { // angle is negative (to the right) or zero
                    adjustAngle = Math.max(-5, currentBearing);
                }
                // turn the robot 5 degrees or to the bearing required, whichever is smaller
                robotDriver.gyroTurn(0.1, adjustAngle, 3);
                tempAprilTag = aprilTagDetector.scanForAprilTagById(focusId); // rescan for april tags
                if (tempAprilTag == null) { // if april tag was not detected, that's ok; we use the previous values instead
                    currentBearing -= adjustAngle;
                }
                else { // the april tag was detected; update with the new info
                    focusedAprilTag = tempAprilTag;
                    currentBearing = focusedAprilTag.ftcPose.bearing;
                    currentRange = focusedAprilTag.ftcPose.range;
                    currentYaw = focusedAprilTag.ftcPose.yaw;
                }
            }
            while (focusedAprilTag.ftcPose.range > 5) {

                // drive the robot 5 inches forward or 5 inches in front of the backdrop; whichever comes first
                robotDriver.gyroDrive(0.3, Math.min(currentRange - 5, 5), 0, 3, null);

                tempAprilTag = aprilTagDetector.scanForAprilTagById(focusId); // rescan for april tags
                if (tempAprilTag == null) { // if april tag was not detected, that's ok; we use the previous values instead
                    currentRange -= Math.min(currentRange - 5, 5);
                }
                else { // the april tag was detected; update with the new info
                    focusedAprilTag = tempAprilTag;
                    currentRange = focusedAprilTag.ftcPose.range;
                    currentYaw = focusedAprilTag.ftcPose.yaw;
                }
            }
            if (Math.abs(focusedAprilTag.ftcPose.yaw) > 5) {
                robotDriver.gyroTurn(0.1, currentYaw, 3); // turn the robot to face the april tag
            }
        }
    }
}

