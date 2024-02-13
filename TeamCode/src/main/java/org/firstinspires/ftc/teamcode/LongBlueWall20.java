package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
public class LongBlueWall20 extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        //robot, dt motors, vision processing setup
        Robot robot = new Robot(hardwareMap, this, telemetry, true, false, true);
        robot.setUpDrivetrainMotors();
        robot.setUpIntakeOuttake();
        robot.initVisionProcessing();
        double slideStartingPosition;
        int delay = 12000;  // Max delay is 12 seconds, leaves 3 seconds for vision timeout


        waitForStart();

        while (opModeIsActive()) {

            robot.detectMarkerPosition();
            robot.visionPortal.setProcessorEnabled(robot.markerProcessor, false);
            robot.visionPortal.setProcessorEnabled(robot.aprilTagProcessor, false);

            robot.setMarkerLocation(false, true, robot.markerPos);
            robot.servoToInitPositions();

            this.sleep(delay);

            robot.longMoveToBoardTruss();

            robot.alignToBoardFast(robot.wantedAprTagId);
            robot.visionPortal.setProcessorEnabled(robot.aprilTagProcessor, false);

            // note slide init position
            slideStartingPosition = robot.lsFront.getCurrentPosition();
            robot.autoOuttake(false, slideStartingPosition);

            // parking here
            robot.boardToTruss();
            robot.straightBlocking2(-10);
            break;

        }
    }
}