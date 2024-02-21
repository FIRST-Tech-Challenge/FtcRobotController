package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
public class BlueLongWall20 extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        //robot, dt motors, vision processing setup
        Robot robot = new Robot(hardwareMap, this, telemetry, true, false, true);
        robot.setUpDrivetrainMotors();
        robot.setUpIntakeOuttake();
        robot.initVisionProcessing();
        double slideStartingPosition;
        int maxDelayInSeconds = 12;
        int delay = 12000;  // Max delay is 12 seconds, leaves 3 seconds for vision timeout


        waitForStart();

        while (opModeIsActive()) {

            robot.detectMarkerPosition();
            robot.visionPortal.setProcessorEnabled(robot.markerProcessor, false);
            robot.visionPortal.setProcessorEnabled(robot.aprilTagProcessor, false);

            robot.setMarkerLocation(false, true, robot.markerPos);
            robot.servoToInitPositions();

            if (robot.autoDelayInSeconds <= maxDelayInSeconds) {
                this.sleep(robot.autoDelayInSeconds * 1000);
            } else {
                this.sleep(maxDelayInSeconds * 1000);
            }

            robot.longMoveToBoardTruss();

            robot.alignToBoardFast(robot.wantedAprTagId);
            robot.visionPortal.setProcessorEnabled(robot.aprilTagProcessor, false);

            // note slide init position
            slideStartingPosition = robot.lsFront.getCurrentPosition();
            robot.autoOuttake(false, slideStartingPosition);

            if (robot.parkFreeway) {
                robot.boardToMiddle(slideStartingPosition);
                robot.straightBlocking2(-10);
            } else {
                robot.boardToTruss(slideStartingPosition);
                robot.straightBlocking2(-10);
            }

        }
    }
}