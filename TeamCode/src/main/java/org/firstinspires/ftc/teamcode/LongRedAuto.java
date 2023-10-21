package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.List;

@Autonomous
public class LongRedAuto extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        boolean isDoneWithAprilTagX = false;
        boolean isDoneWithAprilTagRange = false;
        int idNumber = 1;

        // Initializing marker and apriltag processors and setting them with visionportal
        MarkerProcessor markerProcessor = new MarkerProcessor(telemetry);
        AprilTagProcessor aprilTagProcessor = AprilTagProcessor.easyCreateWithDefaults();
        VisionPortal visionPortal = VisionPortal.easyCreateWithDefaults
                (hardwareMap.get(WebcamName.class, "Webcam 1"),
                        aprilTagProcessor,
                        markerProcessor);

        int cameraMonitorViewId =
                hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        OpenCvWebcam webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        Robot robot = new Robot(hardwareMap, this, telemetry);
        robot.setUpDrivetrainMotors();

        //MarkerDetector detector = new MarkerDetector(telemetry);
        //webcam.setPipeline(detector);
        //webcam.openCameraDevice();
        //webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
        MarkerDetector.MARKER_POSITION position;

        waitForStart();
        telemetry.addData("camera state", String.valueOf(visionPortal.getCameraState()));
        telemetry.update();

        while (!isStarted() || opModeIsActive()) {
            position = markerProcessor.getPosition();

            while (opModeIsActive()) {
                List<AprilTagDetection> myAprilTagDetections = aprilTagProcessor.getDetections();
                telemetry.addLine(String.valueOf(myAprilTagDetections.size()));
                telemetry.update();

                // Cycle through through the list and process each AprilTag
                for (AprilTagDetection detection : myAprilTagDetections) {
                    if (detection.metadata != null) {  // This check for non-null Metadata is not needed for reading only ID code.
                        // Now take action based on this tag's ID code, or store info for later action.
                        telemetry.addData("ID", "%d (%s)", detection.id, detection.metadata.name);
                        telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                        telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                        telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
                    }
                    telemetry.update();
                }
            }


            if (position == MarkerDetector.MARKER_POSITION.CENTER) {
                robot.straightBlocking(20, false);
                robot.setHeading(15);
                robot.straightBlocking(6, false);
                sleep(3000);
                robot.straightBlocking(6, true);
                robot.setHeading(0);
                robot.straightBlocking(19, true);
                break;
            } else if (position == MarkerDetector.MARKER_POSITION.LEFT) {
                robot.straightBlocking(18, false);
                robot.setHeading(30);
                robot.straightBlocking(3, false);
                sleep(3000);
                robot.straightBlocking(3, true);
                robot.setHeading(0);
                robot.straightBlocking(17, true);
                break;
            } else if (position == MarkerDetector.MARKER_POSITION.RIGHT) {
                robot.straightBlocking(14, false);
                robot.setHeading(-45);
                robot.straightBlocking(11, false);
                sleep(3000);
                robot.straightBlocking(11, true);
                robot.setHeading(0);
                robot.straightBlocking(12, true);
                break;
            }
        }
        sleep(100);
        robot.setHeading(0);
        robot.straightBlocking(27, false);
        sleep(100);

        //robot.setUpVisionProcessing();

        /*
        robot.waitFor(0.75);


        robot.straightBlocking(15, false);
        robot.waitFor(0.1);
        robot.setHeading(0);
        robot.waitFor(0.1);
        robot.mecanumBlocking(20, true);
        robot.waitFor(0.1);
        robot.setHeading(0);
        robot.waitFor(0.1);
        robot.straightBlocking(38, false);
        robot.waitFor(0.1);
        robot.setHeading(-90);
        robot.waitFor(0.1);
        robot.straightBlocking(88, false);
        robot.waitFor(0.1);
        robot.setHeading(-90);
        robot.waitFor(0.1);
        robot.mecanumBlocking(28, false);

        //TODO: APRILTAG GOES HERE !!!!!!

        while (opModeIsActive() && !isDoneWithAprilTagX) {


            isDoneWithAprilTagX = robot.moveRelativeToAprilTagX(0, idNumber);


            telemetry.addData("x", isDoneWithAprilTagX);

            telemetry.update();
        }
        telemetry.addLine("got out");
        while (opModeIsActive() && !isDoneWithAprilTagRange) {


                isDoneWithAprilTagRange = robot.moveRelativeToAprilTagRange(10, idNumber);


            telemetry.addData("range", isDoneWithAprilTagRange);

            telemetry.update();
        }


        /*robot.waitFor(2);
        robot.setHeading(0);
        robot.waitFor(0.1);
        robot.straightBlocking(24, true);
        robot.waitFor(0.1);
        robot.setHeading(90);
        robot.waitFor(0.1);
        robot.straightBlocking(84, true);
        robot.waitFor(0.1);
        robot.setHeading(-90);
        robot.waitFor(0.1);
        robot.straightBlocking(84, true);
        robot.waitFor(0.1);
        robot.setHeading(-90);
        robot.waitFor(0.1);
        robot.mecanumBlocking(25, true);
        robot.waitFor(0.1);
*/

    }
}
