package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
public class LongRedAuto extends LinearOpMode {

    Robot robot;

    @Override
    public void runOpMode() throws InterruptedException {

        //robot, dt motors, vision processing setup
        robot = new Robot(hardwareMap, this, telemetry);
        robot.setUpDrivetrainMotors();
        robot.initVisionProcessingRed();

        waitForStart();

        //TODO: unlock slide, clamp down, slide down (not nec in that order)

        robot.detectMarkerPositionRed();
        longRedMoveToBoard();
        robot.alignToBoard();
        robot.setHeading(-90, 0.75);
        robot.autoOuttake();
    }

    public void longRedMoveToBoard() {
        Log.d("vision", "moveToMarker: Pos " + robot.markerPosRed);
        Log.d("vision", "moveToMarker: Tag " + robot.wantedAprTagId);
        while (opModeIsActive()) {
            if (robot.markerPosRed == MarkerDetectorRed.MARKER_POSITION.RIGHT) { //RIGHT
                robot.straightBlocking(19, false, 0.25);
                robot.setHeading(-45, 0.25);
                robot.straightBlocking(8, false, 0.7);
                robot.setHeading(-45, 0.25);
                robot.straightBlocking(12, true, 0.7);
                robot.setHeading(0, 0.7);
                robot.straightBlocking(39, false, 0.7);
                robot.setHeading(-90, 0.7);
                robot.straightBlocking(77, false, 0.7);
                robot.setHeading(-90, 0.75);
                robot.mecanumBlocking(38, false, 0.7);
                robot.setHeading(-90, 0.7);
                robot.mecanumBlocking(15.5, true, 0.5);
                robot.setHeading(-90, 0.7);
                robot.straightBlocking(14, false, 0.5);
                break;
            } else if (robot.markerPosRed == MarkerDetectorRed.MARKER_POSITION.LEFT) { //LEFT
                robot.straightBlocking(18, false, 0.25); //forward
                robot.setHeading(45, 0.25); //turn
                robot.straightBlocking(7, false, 0.7); //forward
                robot.setHeading(45, 0.25);
                robot.straightBlocking(8, true, 0.7); //dropoff, backward
                robot.setHeading(0, 0.7); //turn
                robot.mecanumBlocking(1, false, 0.5); //mecanum right
                robot.setHeading(0, 0.7);
                robot.straightBlocking(33, false, 0.7); //forward to truss
                robot.setHeading(-90, 0.7); //turn
                robot.straightBlocking(72, false, 0.7); //forward to red line
                robot.setHeading(-90, 0.7);
                robot.mecanumBlocking(24, false, 0.7); //mecanum directly in front of board
                robot.setHeading(-90, 0.7);
                break;
            } else { //center, default
                Log.d("vision", "moveToMarker: center or default");
                robot.mecanumBlocking(4, true, 0.5); //go left
                robot.setHeading(0, 0.6);
                robot.straightBlocking(28.5, false, 0.5); //go forward
                robot.setHeading(0, 0.6);
                robot.straightBlocking(6, true, 0.7); //dropoff & move back
                robot.setHeading(0, 0.6);
                robot.mecanumBlocking(12, true, 0.25); //move left
                robot.setHeading(0, 0.7);
                robot.straightBlocking(24, false, 0.7); //go forward & around marker
                robot.setHeading(-90, 0.7); //turn
                robot.straightBlocking(84, false, 0.7); //forward to red line
                robot.setHeading(-90, 0.7);
                robot.mecanumBlocking(29, false,  0.5); //mecanum directly in front of board
                robot.setHeading(-90, 0.7);
                break;
            }
        }
    }
}

