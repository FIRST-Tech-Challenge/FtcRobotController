package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
public class BlueLongWall22 extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        //robot, dt motors, vision processing setup
        Robot robot = new Robot(hardwareMap, this, telemetry, true, false, true);
        robot.setUpDrivetrainMotors();
        robot.setUpIntakeOuttake();
        robot.initVisionProcessing();
        double slideStartingPosition;

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

            // note slide init position
            slideStartingPosition = robot.lsFront.getCurrentPosition();
            robot.autoOuttake(false);

            robot.boardToTruss();
            robot.trussToStackAndIntake();
            robot.visionPortal.setProcessorEnabled(robot.aprilTagProcessor, true);
            robot.stackToBoardTruss();
            robot.intake.setPower(0);

            robot.alignToBoardFast(robot.secondWantedTagId);
            robot.visionPortal.setProcessorEnabled(robot.aprilTagProcessor, false);

            robot.autoOuttake(false);

            break;

        }
    }
}