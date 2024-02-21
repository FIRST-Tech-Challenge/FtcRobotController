package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
public class BlueLongFreeway20 extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        //robot, dt motors, vision processing setup
        int maxDelayInSeconds = 12;
        Robot robot = new Robot(hardwareMap, this, telemetry, true, false, true);
        robot.setDelayAndParking(maxDelayInSeconds, Robot.PARKING_POSITION.FREEWAY);
        robot.buttonConfigAtInit(gamepad1);

        robot.setUpDrivetrainMotors();
        robot.setUpIntakeOuttake();
        robot.initVisionProcessing();
        robot.slideStartingPosition = robot.lsFront.getCurrentPosition();

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

            robot.longMoveToBoard(false);
            robot.alignToBoardFast(robot.wantedAprTagId);
            robot.visionPortal.setProcessorEnabled(robot.aprilTagProcessor, false);

            // note slide init position
            robot.autoOuttake(false);

            robot.configuredParking();

            break;

        }
    }
}
