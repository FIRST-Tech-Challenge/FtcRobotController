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

        //robot, dt motors, vision processing setup
        Robot robot = new Robot(hardwareMap, this, telemetry);
        robot.setUpDrivetrainMotors();
        robot.initVisionProcessing();

        //vision processing stuff
        /*
        VisionPortal visionPortal = robot.getVisionPortal();

        MarkerProcessor markerProcessor = robot.getMarkerProcessor();
        AprilTagProcessor aprilTagProcessor = robot.getAprilTagProcessor();

        MarkerDetector.MARKER_POSITION position;
        */

        sleep(3000);
        waitForStart();

        while (opModeIsActive()) {

            //detectmarkerposition() code commented below
            /*
            //detect marker position
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
            */

            robot.detectMarkerPosition();
            robot.moveToMarker();

            sleep(1000);

            //move to backdrop from spike marks
            robot.setHeading(0, 0.75);
            sleep(100);
            robot.mecanumBlocking(20, true, 0.5);
            sleep(100);
            robot.setHeading(0, 0.75);
            sleep(100);
            robot.straightBlocking(35, false, 0.7);
            sleep(100);
            robot.setHeading(-90, 0.75);
            sleep(100);
            robot.straightBlocking(90, false, 0.7);
            sleep(100);
            robot.setHeading(-90, 0.5);
            sleep(100);
            robot.mecanumBlocking(28, false, 0.5);
            sleep(100);
            robot.setHeading(-90, 0.75);

            sleep(2000);

            robot.moveToBoard();
            sleep(100);
            robot.setHeading(-90, 0.75);

            break;

            //movetoboard() code commented below
        }
    }
}

