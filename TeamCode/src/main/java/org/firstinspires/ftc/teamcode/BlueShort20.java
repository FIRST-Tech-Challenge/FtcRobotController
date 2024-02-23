package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
public class BlueShort20 extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        //robot, dt motors, vision processing setup
        int maxDelayInSeconds = 11;
        Robot robot = new Robot(hardwareMap, this, telemetry, false, false, true);
        robot.setUpDrivetrainMotors();
        robot.setUpIntakeOuttake();
        robot.initVisionProcessing();
        double slideStartingPosition;
        int delay = 0;  // Max delay is 12000
        robot.setConfigPresets(delay, Robot.PARKING_POSITION.TRUSS, true );

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

            // note slide init position
            slideStartingPosition = robot.lsFront.getCurrentPosition();

            robot.autoOuttake(robot.lowOuttake);

            // parking here
            robot.configuredParking();

            break;
        }
    }
}