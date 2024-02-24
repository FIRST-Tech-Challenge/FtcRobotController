package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
public class BlueShortWall22 extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        Robot robot = new Robot(hardwareMap, this, telemetry, false, false, true);
        robot.setUpDrivetrainMotors();
        robot.setUpIntakeOuttake();
        robot.initVisionProcessing();

        waitForStart();

        while (opModeIsActive()) {

            robot.detectMarkerPosition();
            robot.visionPortal.setProcessorEnabled(robot.markerProcessor, false);
            robot.visionPortal.setProcessorEnabled(robot.aprilTagProcessor, false);

            robot.setMarkerLocation(false, false, robot.markerPos);
            robot.servoToInitPositions();

            robot.shortMoveToBoard2();

            robot.alignToBoardFast(robot.wantedAprTagId);
            robot.visionPortal.setProcessorEnabled(robot.aprilTagProcessor, false);

            robot.autoOuttake(true);

            robot.boardToTruss();

            robot.trussToStackAndIntake();
            robot.visionPortal.setProcessorEnabled(robot.aprilTagProcessor, true);
            robot.stackToBoardTruss();
            robot.intake.setPower(0);

            robot.alignToBoardFast(robot.secondWantedTagId);
            robot.autoOuttake(false);

            robot.visionPortal.setProcessorEnabled(robot.aprilTagProcessor, false);

            break;
        }
    }
}