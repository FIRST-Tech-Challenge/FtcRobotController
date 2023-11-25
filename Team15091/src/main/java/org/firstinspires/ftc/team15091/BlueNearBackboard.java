package org.firstinspires.ftc.team15091;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

@Autonomous(name = "BlueNearBackboard", preselectTeleOp = "Gamepad")
public class BlueNearBackboard extends AutonomousBase{
    // arm servo: 0 is out, 0.8 is in
    // bowl servo: 1 is out, 0 is in
    Thread armUp = new Thread() {
        public void run() {
            robot.liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.liftMotor.setTargetPosition(400);
            robot.liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            while(robot.liftMotor.isBusy()) {
                robot.liftMotor.setPower(0.8);
                idle();
            }
            robot.liftMotor.setPower(0);
        }
    };

    Thread armDown = new Thread() {
        public void run() {
            robot.liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.liftMotor.setTargetPosition(0);
            robot.liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            while(robot.liftMotor.isBusy()) {
                robot.liftMotor.setPower(0.8);
                idle();
            }
            robot.liftMotor.setPower(0);
        }
    };
    @Override
    public void runOpMode() throws InterruptedException {
        setupAndWait();
        DistanceDetector frontDistance = new DistanceDetector((DistanceSensor)(hardwareMap.get("sensor_front")), 7, false);
        // DistanceDetector frontDistanceShort = new DistanceDetector((DistanceSensor)(hardwareMap.get("sensor_front")), 5, false);
        PixelPosition initialPos = rbProcessor.position;
        robot.setArmPosition(0.7);
        if (initialPos == PixelPosition.Left) {
            robotDriver.gyroDrive(0.3d, 12d, 0, 3, null);
            robotDriver.gyroDrive(0.2d, 24.5d, 45, 3, null);
            // robotDriver.gyroDrive(0.3, 3, 45, 3, null); // move forward
            robot.togglePixelHolder(true); // release pixel
            sleep(500);
            robotDriver.gyroDrive(0.3, -10, 45, 3, null); // move backward
            armUp.run();
            robotDriver.gyroTurn(0.2, 90, 3);
            // robotDriver.gyroSlide(0.3, 10, 90, 3, null); // slide one tile to the left
            robotDriver.gyroDrive(0.3, 22.5, 90, 3, null);
            robotDriver.gyroDrive(0.2, 22.5, 90, 3, frontDistance);
            telemetry.update();
            //robotDriver.gyroDrive(0.1, 3, 90, 3, distanceToBoard);
            robotDriver.gyroSlide(0.1, -5, 90, 3, null);
            robot.armServo.setPosition(0);
            sleep(1500);
            robot.setBowlPosition(0.45);
            sleep(1500);
            robotDriver.gyroDrive(0.2, -5, 90, 3, null);
            robot.armServo.setPosition(0.8d);
            robot.setBowlPosition(0);
            sleep(200);
            armDown.run();
            robotDriver.gyroSlide(0.3, 17.5, 90, 3, null);
            robotDriver.gyroDrive(0.3, 22.5, 90,  3, frontDistance);
        }
        else if (initialPos == PixelPosition.Right) {
            robotDriver.gyroDrive(0.3d, 12d, 0, 3, null);
            robotDriver.gyroDrive(0.2d, 25d, -45, 3, null);
            robot.togglePixelHolder(true); // release pixel
            sleep(500);
            robotDriver.gyroDrive(0.3, -10, -45, 3, null); // move backward
            armUp.run();
            robotDriver.gyroTurn(0.2, 90, 3);
            robotDriver.gyroDrive(0.3, 22.5, 90, 3, null);
            robotDriver.gyroDrive(0.2, 22.5, 90, 3, frontDistance);
            robotDriver.gyroSlide(0.2, -15,90, 5, null);
            robot.armServo.setPosition(0);
            sleep(1500);
            robot.setBowlPosition(0.45);
            sleep(1500);
            robotDriver.gyroDrive(0.2, -5, 90, 3, null);
            robot.armServo.setPosition(0.8d);
            robot.setBowlPosition(0);
            sleep(200);
            armDown.run();
            robotDriver.gyroSlide(0.3, 34, 90, 5, null);
            robotDriver.gyroDrive(0.3, 22.5, 90,  5, frontDistance);
        }
        else { // pixel in the middle position
            robotDriver.gyroDrive(0.2, 29.5, 0, 5, null);
            robotDriver.gyroDrive(0.2, -7, 0, 5, null);
            armUp.run();
            robotDriver.gyroTurn(0.1, 90, 5);
            robotDriver.gyroDrive(0.3, 22.5, 90, 5, null);
            robotDriver.gyroDrive(0.2, 22.5, 90, 5, frontDistance);
            robotDriver.gyroSlide(0.2, -10, 90, 5, null);
            robot.armServo.setPosition(0);
            sleep(1500);
            robot.setBowlPosition(0.45);
            sleep(1500);
            robotDriver.gyroDrive(0.2, -5, 90, 3, null);
            robot.armServo.setPosition(0.8d);
            robot.setBowlPosition(0);
            sleep(200);
            armDown.run();
            robotDriver.gyroSlide(0.3, 29, 90, 5, null);
            robotDriver.gyroDrive(0.3, 22.5, 90,  5, frontDistance);
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

