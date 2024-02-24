package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.nio.file.ClosedWatchServiceException;

@Autonomous
public class BlueShortFreeway22 extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        //robot, dt motors, vision processing setup
        Robot robot = new Robot(hardwareMap, this, telemetry, false, false, true);
        robot.setUpDrivetrainMotors();
        robot.setUpIntakeOuttake();
        robot.initVisionProcessing();
        int defaultDelay = 0;  // Max delay is 3000
        int maxDelayInSeconds = 3;


        robot.setConfigPresets(defaultDelay, Robot.PARKING_POSITION.BOARD, true );

        robot.buttonConfigAtInit(gamepad1);

        waitForStart();

        while (opModeIsActive()) {

            robot.detectMarkerPosition();
            robot.visionPortal.setProcessorEnabled(robot.markerProcessor, false);
            robot.visionPortal.setProcessorEnabled(robot.aprilTagProcessor, false);

            robot.setMarkerLocation(false, false, robot.markerPos);
            robot.servoToInitPositions();

            if (robot.autoDelayInSeconds <= maxDelayInSeconds) {
                this.sleep(robot.autoDelayInSeconds * 1000);
            } else {
                this.sleep(maxDelayInSeconds * 1000);
            }

            robot.shortMoveToBoard2();
            robot.alignToBoardFast(robot.wantedAprTagId);
            robot.visionPortal.setProcessorEnabled(robot.aprilTagProcessor, false);

            robot.autoOuttake(true);

            robot.boardToMiddle();
            robot.middleToStackAndIntake();
            robot.visionPortal.setProcessorEnabled(robot.aprilTagProcessor, true);
            robot.stackToBoard();

            robot.alignToBoardFast(robot.secondWantedTagId);
            robot.visionPortal.setProcessorEnabled(robot.aprilTagProcessor, false);

            robot.trayToOuttakePos(true); // pivot tray to outtake position
            robot.autoOuttake(false);

            break;

        }
    }
}