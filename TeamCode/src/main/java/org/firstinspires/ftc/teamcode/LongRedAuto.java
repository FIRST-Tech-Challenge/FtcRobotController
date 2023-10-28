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

        //robot, dt motors
        Robot robot = new Robot(hardwareMap, this, telemetry);
        robot.setUpDrivetrainMotors();

        //vision processing
        robot.initVisionProcessing();
        VisionPortal visionPortal = robot.getVisionPortal();
        MarkerProcessor markerProcessor = robot.getMarkerProcessor();
        AprilTagProcessor aprilTagProcessor = robot.getAprilTagProcessor();

        MarkerDetector.MARKER_POSITION position;

        waitForStart();
        Log.d("vision", "camera state: " + visionPortal.getCameraState());

        while (opModeIsActive()) {

            //detect marker position FIX RESOLUTION IN DETECTOR BOXES
            position = markerProcessor.getPosition();

            while (position == MarkerDetector.MARKER_POSITION.UNDETECTED) {
                Log.d("vision", "undetected marker, keep looking");
                position = markerProcessor.getPosition();
            }

            //print position
            Log.d("vision", "detected position: " + position);

            //save marker position, apriltag position
            robot.setMarkerPos(position);
            robot.setWantedAprTagId(position, true);

            //move to marker
            robot.moveToMarker();
            sleep(1000);

            //move to board from spike marks
            robot.straightBlocking(15, false);
            sleep(1000);
            robot.setHeading(0);
            sleep(1000);
            robot.mecanumBlocking(20, true);
            sleep(1000);
            robot.setHeading(0);
            sleep(1000);
            robot.straightBlocking(38, false);
            sleep(1000);
            robot.setHeading(-90);
            sleep(1000);
            robot.straightBlocking(90, false);
            sleep(1000);
            robot.setHeading(-90);
            sleep(1000);
            robot.mecanumBlocking(28, false);
            sleep(1000);
            robot.setHeading(-90);

            sleep(2000);

            robot.moveToBoard();

            /*
            int desiredAprTagId = robot.wantedAprTagId; //test 5
            boolean tagVisible = false;
            boolean aligned = false;
            List<AprilTagDetection> myAprilTagDetections;

            while (opModeIsActive()) {

                while (!aligned) {

                    myAprilTagDetections = aprilTagProcessor.getDetections();

                    // Cycle through through the list and process each AprilTag

                    for (AprilTagDetection detection : myAprilTagDetections) {
                        if (detection.metadata != null) {  // This check for non-null Metadata is not needed for reading only ID code.
                            // Now store info for later action.
                            telemetry.addLine(String.valueOf(myAprilTagDetections.size()));
                            telemetry.addData("ID", "%d (%s)", detection.id, detection.metadata.name);
                            telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                            telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                            telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));

                            if (detection.id == desiredAprTagId) {
                                Log.d("vision", "runOpMode: tag visible - this one " + desiredAprTagId);
                                tagVisible = true;

                                if (detection.ftcPose.bearing > 1) {
                                    Log.d("vision", "runOpMode: bearing > 1, move left");
                                    robot.mecanumBlocking(1, true);
                                    sleep(100);
                                } else if (detection.ftcPose.bearing < -1) {
                                    Log.d("vision", "runOpMode: bearing < -1, move right");
                                    robot.mecanumBlocking(1, false);
                                    sleep(100);
                                } else {
                                    Log.d("vision", "runOpMode: aligned");
                                    aligned = true;
                                }

                                break;
                            }
                            telemetry.update();
                        }

                        if (!tagVisible) {
                            Log.d("vision", "runOpMode: tag not visible, move back");
                            robot.straightBlocking(1, true);
                            sleep(100);
                        }
                    }
                }

                sleep(100);

                myAprilTagDetections = aprilTagProcessor.getDetections();
                for (AprilTagDetection detection : myAprilTagDetections) {
                    if (detection.metadata != null) {  // This check for non-null Metadata is not needed for reading only ID code.
                        // Now store info for later action.
                        if (detection.id == desiredAprTagId) {
                            Log.d("vision", "runOpMode: bearing is " + detection.ftcPose.bearing);
                        }

                        double distanceToBoard = detection.ftcPose.range - 5;
                        Log.d("vision", "runOpMode: distance is " + distanceToBoard);
                        robot.straightBlocking(distanceToBoard, false);
                        sleep(1000);
                        break;
                    }
                }
            }
            */
        }
    }
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
