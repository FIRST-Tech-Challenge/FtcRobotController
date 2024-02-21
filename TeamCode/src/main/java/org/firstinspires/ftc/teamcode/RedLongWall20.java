package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
public class RedLongWall20 extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        //robot, dt motors, vision processing setup
        int maxDelayInSeconds = 12;
        Robot robot = new Robot(hardwareMap, this, telemetry, true, true, true);
        robot.setDelayAndParking(maxDelayInSeconds, false);

        robot.setUpDrivetrainMotors();
        robot.setUpIntakeOuttake();
        robot.initVisionProcessing();
        double slideStartingPosition;

        waitForStart();

        while (opModeIsActive()) {

            robot.detectMarkerPosition();
            robot.visionPortal.setProcessorEnabled(robot.markerProcessor, false);
            robot.visionPortal.setProcessorEnabled(robot.aprilTagProcessor, false);

            robot.setMarkerLocation(true, true, robot.markerPos);
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

            break;

        }
    }
}