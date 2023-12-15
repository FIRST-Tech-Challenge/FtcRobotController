package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
public class ShortRedAuto extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        //robot, dt motors, vision processing setup
        Robot robot = new Robot(hardwareMap, this, telemetry, true, true);
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
            robot.alignToBoard();
            robot.autoOuttake(true, slideStartingPosition);
            robot.parkBot(false);

            //second trip
            /*
            robot.mecanumBlocking(23,true, 0.7); // mecanum directly in front of board left if blue
            robot.setHeading(-90, 0.7);
            robot.stackAttachmentOut(); //stack attachment out
            robot.fastStraightFixHeading(107, true, 1); // move forward while stack attachment is moving
            robot.mecanumBlocking(11, true, 0.3); // strafe to knock over stack
            robot.stackAttachmentIn(); // attachment in
            robot.mecanumBlocking(11, false, 0.7); // move back to knocked stack
            robot.autoIntake(); //intake
            robot.closeClamp(true);
            robot.fastStraightFixHeading(100, false, 1); // drive across field
            robot.mecanumBlocking(25, false, 0.7); // mecanum to board

            // move linear slide up
            robot.moveLinearSlideByTicksBlocking(2000 + slideStartingPosition);

            robot.trayToOuttakePos(true); // pivot tray to outtake position
            robot.fastStraightFixHeading(12, false, 0.7);
            //robot.alignToBoard();

            robot.autoOuttake(false, slideStartingPosition);
            */


            break;
        }
    }
}

