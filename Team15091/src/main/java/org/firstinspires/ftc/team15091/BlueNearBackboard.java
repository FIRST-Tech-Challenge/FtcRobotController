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
            robot.liftMotor.setTargetPosition(90);
            robot.liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            while(robot.liftMotor.isBusy()) {
                robot.liftMotor.setPower(0.8);
                idle();
            }
            robot.liftMotor.setPower(0);
            robot.armServo.setPosition(0);
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
        // DistanceDetector frontDistanceShort = new DistanceDetector((DistanceSensor)(hardwareMap.get("sensor_front")), 1, false);
        PixelPosition initialPos = rbProcessor.position;
        boolean aprilTagFound = false;
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
            if (aprilTagDetector.scanForAprilTagById(1) != null) { // scan for tag
                positionToAprilTag(1);
                aprilTagFound = true;
                robot.beep(0);
            }
            else {
                robotDriver.gyroDrive(0.2, 22.5, 90, 3, frontDistance);
                robotDriver.gyroSlide(0.1, -5, 90, 3, null);
                // robotDriver.gyroDrive(0.1, 3, 90, 3, distanceToBoard);
            }
            frontDistance.setThreshold(1);
            robotDriver.gyroDrive(0.1, 7, 90, 5, frontDistance);
            // robotDriver.gyroSlide(0.1, -3, 90, 3, null); // slide to the right -- in testing this gives a better pixel "bounce"
            sleep(1000);
            robot.setBowlPosition(1);
            sleep(1000);
            robotDriver.gyroDrive(0.2, -5, 90, 3, null);
            robot.armServo.setPosition(0.8d);
            robot.setBowlPosition(0.5);
            sleep(200);
            armDown.run();
            robotDriver.gyroSlide(0.3, 22.5, 90, 3, null);
            frontDistance.setThreshold(7);
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
            for (int i = 0; i < 5; i++) { // repeatedly scan for april tags while moving right
                if (aprilTagDetector.scanForAprilTagById(3) != null) { // tag found
                    positionToAprilTag(3);
                    aprilTagFound = true;
                    robot.beep(0);
                    break;
                }
                robotDriver.gyroSlide(0.2, -3, 90, 5, null);
            }
            if (!aprilTagFound) { // tag not found, use distance sensor instead
                robotDriver.gyroDrive(0.2, 22.5, 90, 5, frontDistance);
            }
            frontDistance.setThreshold(1);
            robotDriver.gyroDrive(0.1, 7, 90, 5, frontDistance);
            sleep(1500);
            robot.setBowlPosition(0);
            sleep(1500);
            robotDriver.gyroDrive(0.2, -5, 90, 3, null);
            robot.armServo.setPosition(0.8d);
            robot.setBowlPosition(0.5);
            sleep(200);
            armDown.run();
            robotDriver.gyroSlide(0.3, 34, 90, 5, null);
            frontDistance.setThreshold(7);
            robotDriver.gyroDrive(0.3, 22.5, 90,  5, frontDistance);
        }
        else { // pixel in the middle position
            robotDriver.gyroDrive(0.2, 29.5, 0, 5, null);
            robotDriver.gyroDrive(0.2, -7, 0, 5, null);
            armUp.run();
            robotDriver.gyroTurn(0.1, 90, 5);
            robotDriver.gyroDrive(0.3, 22.5, 90, 5, null);

            for (int i = 0; i < 4; i++) { // repeatedly scan for april tags while moving right
                if (aprilTagDetector.scanForAprilTagById(2) != null) { // tag found
                    positionToAprilTag(2);
                    aprilTagFound = true;
                    robot.beep(0);
                    break;
                }
                robotDriver.gyroSlide(0.2, -2, 90, 5, null);
            }
            if (!aprilTagFound) { // tag not found, use distance sensor instead
                robotDriver.gyroDrive(0.2, 22.5, 90, 5, frontDistance);
            }
            robotDriver.gyroSlide(0.1, -1, 90, 3, null);
            frontDistance.setThreshold(1);
            robotDriver.gyroDrive(0.1, 6, 90, 5, frontDistance);
            sleep(200);
            robot.setBowlPosition(0.75);
            sleep(1000);
            robot.setBowlPosition(1);
            sleep(700);
            robotDriver.gyroDrive(0.2, -5, 90, 3, null);
            robot.armServo.setPosition(0.8d);
            robot.setBowlPosition(0.5);
            sleep(200);
            armDown.run();
            robotDriver.gyroSlide(0.3, 29, 90, 5, null);
            frontDistance.setThreshold(7);
            robotDriver.gyroDrive(0.3, 22.5, 90,  5, frontDistance);
        }
    }

    /**
     * Drives to a detected April Tag.
     * @param focusId The ID of the tag to drive to, if detected by the camera.
     */
    private void positionToAprilTag (int focusId) {
        AprilTagDetection focusedAprilTag = aprilTagDetector.scanForAprilTagById(focusId);
        double adjustDistance;
        if (focusedAprilTag != null) {
            AprilTagDetection tempAprilTag;
            double currentBearing = focusedAprilTag.ftcPose.bearing;
            double currentRange = focusedAprilTag.ftcPose.range;
            double currentYaw = focusedAprilTag.ftcPose.yaw;
            do { // while the robot is not facing the correct bearing
                adjustDistance = currentRange * Math.asin(currentBearing * Math.PI / 180);
                if (currentBearing > 0) { // angle is positive (to the left)
                    adjustDistance = -adjustDistance - 1.2;
                } else { // angle is negative (to the right) or zero
                    adjustDistance = adjustDistance + 1.2;
                }
                telemetry.addLine("April Tag Positioning |")
                        .addData("Bearing", "%.2f", currentBearing)
                        .addData("Distance", "%.2f", adjustDistance);
                telemetry.update();
                robotDriver.gyroSlide(0.2, adjustDistance, 90, 5, null);
                // robotDriver.gyroTurn(0.1, robot.getHeading() + adjustAngle, 3);
                tempAprilTag = aprilTagDetector.scanForAprilTagById(focusId); // rescan for april tags
                if (tempAprilTag != null) { // the april tag was detected; update with the new info
                    focusedAprilTag = tempAprilTag;
                    currentBearing = focusedAprilTag.ftcPose.bearing;
                    currentRange = focusedAprilTag.ftcPose.range;
                    currentYaw = focusedAprilTag.ftcPose.yaw;
                    adjustDistance = currentRange * Math.asin(currentBearing * Math.PI / 180);
                    if (currentBearing > 0) { // angle is positive (to the left)
                        adjustDistance = -adjustDistance - 1.2;
                    } else { // angle is negative (to the right) or zero
                        adjustDistance = adjustDistance + 1.2;
                    }
                }
                else {
                    adjustDistance = 0;
                }
            } while (Math.abs(adjustDistance)> 2);
            //}
            while (currentRange > 5) {

                // drive the robot 5 inches forward or 5 inches in front of the backdrop; whichever comes first
                robotDriver.gyroDrive(0.3, Math.min(currentRange - 5, 5), 90, 5, null);

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
            /*if (Math.abs(focusedAprilTag.ftcPose.yaw) > 5) {
                robotDriver.gyroTurn(0.1, currentYaw - robot.getHeading(), 3); // turn the robot to face the april tag
            }*/
        }
    }
}

