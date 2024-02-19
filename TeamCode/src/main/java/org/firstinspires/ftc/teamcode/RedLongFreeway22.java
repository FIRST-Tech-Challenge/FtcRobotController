package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
public class RedLongFreeway22 extends LinearOpMode {
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
            robot.visionPortal.setProcessorEnabled(robot.markerProcessor, false);
            robot.visionPortal.setProcessorEnabled(robot.aprilTagProcessor, false);

            robot.setMarkerLocation(true, true, robot.markerPos);
            robot.servoToInitPositions();

            robot.longMoveToBoard(false);
            robot.alignToBoardFast(robot.wantedAprTagId);

            robot.visionPortal.setProcessorEnabled(robot.aprilTagProcessor, false);

            // note slide init position
            slideStartingPosition = robot.lsFront.getCurrentPosition();
            robot.autoOuttake(false, slideStartingPosition);

            robot.boardToMiddle(slideStartingPosition);
            robot.middleToStackAndIntake();

            robot.visionPortal.setProcessorEnabled(robot.aprilTagProcessor, true);
            robot.stackToBoard();
            robot.intake.setPower(0);
            robot.alignToBoardFast(robot.secondWantedTagId);
            robot.visionPortal.setProcessorEnabled(robot.aprilTagProcessor, false);

            robot.trayToOuttakePos(true); // pivot tray to outtake position
            robot.autoOuttake(false, slideStartingPosition);

            break;

        }
    }
}

// todo write timeout for apriltag final forward
// todo how to stop streaming
// todo bring back to board
// todo set complementary tag id
// todo slide not high enough second time
// todo turns need a timeout, and maybe other control loops
// todo tune tray outtake pos/how close it is to board