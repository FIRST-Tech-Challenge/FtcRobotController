package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
public class LongRedAuto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        //robot, dt motors, vision processing setup
        Robot robot = new Robot(hardwareMap, this, telemetry, true, true, true);
        robot.setUpDrivetrainMotors();
        robot.setUpIntakeOuttake();
        robot.initVisionProcessing();
        double slideStartingPosition;

        waitForStart();

        while (opModeIsActive()) {
            robot.detectMarkerPosition();
            robot.longMoveToBoard(false);

            robot.closeClamp(false);
            robot.openHook();
            robot.setServoPosBlocking(robot.spikeServo, 0.5);
            sleep(100);

            slideStartingPosition = robot.lsFront.getCurrentPosition() + 50; //fake zero = 50 so slides don't slam down

            // move linear slide up
            robot.moveLinearSlideByTicksBlocking(2000 + slideStartingPosition);

            robot.trayToOuttakePos(true); // pivot tray to outtake position

            robot.markerPos = MarkerDetector.MARKER_POSITION.CENTER;
            robot.wantedAprTagId = 5;
            robot.alignToBoardFast();
            robot.autoOuttake(false, slideStartingPosition);
            //robot.parkBot(true);

            break;
        }
    }
}

