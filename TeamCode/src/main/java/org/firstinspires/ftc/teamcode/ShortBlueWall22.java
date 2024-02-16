package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
public class ShortBlueWall22 extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        //robot, dt motors, vision processing setup
        Robot robot = new Robot(hardwareMap, this, telemetry, false, false, true);
        robot.setUpDrivetrainMotors();
        robot.setUpIntakeOuttake();
        robot.initVisionProcessing();
        double slideStartingPosition;

        waitForStart();

        while (opModeIsActive()) {

            robot.setMarkerPos(MarkerDetector.MARKER_POSITION.RIGHT);
            robot.setWantedAprTagId(MarkerDetector.MARKER_POSITION.RIGHT, robot.isRedAlliance ? MarkerDetector.ALLIANCE_COLOR.RED : MarkerDetector.ALLIANCE_COLOR.BLUE);
            robot.setSecondWantedTagId();

            /*
            robot.detectMarkerPosition();
            robot.visionPortal.setProcessorEnabled(robot.markerProcessor, false);
            robot.visionPortal.setProcessorEnabled(robot.aprilTagProcessor, false);
            */

            robot.setMarkerLocation(false, false, robot.markerPos);
            robot.servoToInitPositions();

            /*
            robot.shortMoveToBoard2();

            robot.alignToBoardFast(robot.wantedAprTagId);
            robot.visionPortal.setProcessorEnabled(robot.aprilTagProcessor, false);
            */

            // note slide init position
            slideStartingPosition = robot.lsFront.getCurrentPosition();

            /*
            robot.autoOuttake(true, slideStartingPosition);

            robot.boardToTruss();

            sleep(5000);
            */

            robot.trussToStackAndIntake();
            robot.visionPortal.setProcessorEnabled(robot.aprilTagProcessor, true);
            robot.stackToBoardTruss();
            robot.intake.setPower(0);

            robot.alignToBoardFast(robot.secondWantedTagId);
            robot.autoOuttake(false, slideStartingPosition);

            robot.visionPortal.setProcessorEnabled(robot.aprilTagProcessor, false);

            break;
        }
    }
}