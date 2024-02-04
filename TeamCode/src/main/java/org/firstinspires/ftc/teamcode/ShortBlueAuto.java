package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
public class ShortBlueAuto extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        //robot, dt motors, vision processing setup
        Robot robot = new Robot(hardwareMap, this, telemetry, false, false, true);
        robot.setUpDrivetrainMotors();
        robot.setUpIntakeOuttake();
        robot.initVisionProcessing();
        double slideStartingPosition;

        robot.detectPropEarly();

        waitForStart();

        while (opModeIsActive()) {
            if (robot.markerPos == MarkerDetector.MARKER_POSITION.UNDETECTED ||
                    robot.markerPos == MarkerDetector.MARKER_POSITION.UNKNOWN) {
                robot.detectMarkerPosition();
                Log.d("early vision", "auto: ran detectMarkerPosition(). prop undetected/unknown at start");
            }
            robot.shortMoveToBoard();

            slideStartingPosition = robot.lsFront.getCurrentPosition() + 50; //fake zero = 50 so slides don't slam down

            // move linear slide up
            robot.moveLinearSlideByTicksBlocking(1550 + slideStartingPosition);

            robot.trayToOuttakePos(true); // pivot tray to outtake position
            robot.alignToBoardFast(robot.wantedAprTagId);
            robot.autoOuttake(true, slideStartingPosition);
            robot.parkBot(false);

            break;
        }
    }
}

