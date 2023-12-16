package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
public class SecondPathBlue extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        //robot, dt motors, vision processing setup
        Robot robot = new Robot(hardwareMap, this, telemetry, false, true);
        robot.setUpDrivetrainMotors();
        robot.setUpIntakeOuttake();
        robot.initVisionProcessing();
        double slideStartingPosition;
        boolean secondPath = true;

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
            robot.alignToBoard();
            robot.autoOuttake(true, slideStartingPosition);

            //second trip

            robot.boardToCenter();

            // move forward and fast
            robot.stackAttachmentOut(); //stack attachment out
            robot.fastStraightFixHeading(107, true, 1); // move forward while stack attachment is moving

            // remove top 4 pixels
            robot.mecanumBlocking(12, true, 0.5); // strafe to knock over stack
            robot.stackAttachmentIn(); // attachment in
            robot.mecanumBlocking(12, false, 0.5); // move back to knocked stack

            // gobble more pixels
            robot.autoIntake();

            // return to board
            robot.fastStraightFixHeading(92, false, 1); // drive across field
            robot.mecanumBlocking(21, !robot.isRedAlliance, 0.7); // mecanum to board

            // move linear slide up
            robot.moveLinearSlideByTicksBlocking(2000 + slideStartingPosition);
            robot.trayToOuttakePos(true); // pivot tray to outtake position

            // move to board and drop
            robot.goToAnyTag();
            robot.autoOuttake(false, slideStartingPosition);

            break;
        }
    }
}

