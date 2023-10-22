package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@Autonomous
public class LongRedAuto extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        int idNumber = 0;

       Robot robot = new Robot(hardwareMap, this, telemetry);
       robot.setUpDrivetrainMotors();

        robot.initVisionProcessing();
        VisionPortal visionPortal = robot.getVisionPortal();
        MarkerProcessor markerProcessor = robot.getMarkerProcessor();
        AprilTagProcessor aprilTagProcessor = robot.getAprilTagProcessor();

        MarkerDetector.MARKER_POSITION position;

        waitForStart();
        telemetry.addData("camera state", String.valueOf(visionPortal.getCameraState()));
        telemetry.update();

        while (opModeIsActive()) {
            position = markerProcessor.getPosition();

            while (position == MarkerDetector.MARKER_POSITION.UNDETECTED) {
                Log.d("vision", "undetected marker, keep looking");
                position = markerProcessor.getPosition();
            }

            Log.d("vision", "detected position: " + position);
           // idNumber = robot.detectAndMoveToMarker();
            Log.d("vision", "Done moving, now apriltag");
            robot.waitFor(0.75);

/*
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
*/
            /*
            //TODO: APRILTAG GOES HERE !!!!!!
            robot.aprilTagFnaggling(idNumber, 300, 0);
            */

            break;
            /*while (opModeIsActive() && !isDoneWithAprilTagX) {


                isDoneWithAprilTagX = robot.moveRelativeToAprilTagX(0, idNumber);


                telemetry.addData("x", isDoneWithAprilTagX);

                telemetry.update();
            }
            telemetry.addLine("got out");
            while (opModeIsActive() && !isDoneWithAprilTagRange) {


                isDoneWithAprilTagRange = robot.moveRelativeToAprilTagRange(10, idNumber);


                telemetry.addData("range", isDoneWithAprilTagRange);

                telemetry.update();
            }*/


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
        robot.waitFor(0.1);*/

        /*while (opModeIsActive()) {
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
        }*/


        }




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
