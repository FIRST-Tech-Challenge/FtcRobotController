package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
public class BlueLongWall22 extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        int maxDelayInSeconds = 0;
        Robot robot = new Robot(hardwareMap, this, telemetry, true, false, true);
        robot.setConfigPresets(maxDelayInSeconds, Robot.PARKING_POSITION.BOARD, false);

        robot.setUpDrivetrainMotors();
        robot.setUpIntakeOuttake();
        robot.initVisionProcessing();

        waitForStart();

        while (opModeIsActive()) {

            robot.detectMarkerPosition();
            robot.visionPortal.setProcessorEnabled(robot.markerProcessor, false);
            robot.visionPortal.setProcessorEnabled(robot.aprilTagProcessor, false);

            robot.setMarkerLocation(false, true, robot.markerPos);
            robot.servoToInitPositions();

            robot.longMoveToBoardTruss();
            robot.alignToBoardFast(robot.wantedAprTagId);
            robot.visionPortal.setProcessorEnabled(robot.aprilTagProcessor, false);

            robot.autoOuttake(false);

            robot.boardToTruss();
            robot.trussToStackAndIntake();

            robot.visionPortal.setProcessorEnabled(robot.aprilTagProcessor, true);
            robot.stackToBoardTruss();
            robot.intakeMotor.setPower(0);

            robot.alignToBoardFast(robot.secondWantedTagId);
            robot.visionPortal.setProcessorEnabled(robot.aprilTagProcessor, false);

            robot.trayToOuttakePos(true); // pivot tray to outtake position
            robot.autoOuttake(false);

            robot.configuredParking();

            break;

        }
    }
}