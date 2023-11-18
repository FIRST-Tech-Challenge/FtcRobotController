package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
@Disabled
@Autonomous
public class TestingWade extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        Robot robot = new Robot(hardwareMap, this, telemetry, false);
        robot.setUpDrivetrainMotors();
        robot.initVisionProcessing();
        waitForStart();

        while (opModeIsActive()) {
            robot.setMarkerPos(MarkerDetector.MARKER_POSITION.RIGHT);
            robot.setWantedAprTagId(MarkerDetector.MARKER_POSITION.RIGHT, MarkerDetector.ALLIANCE_COLOR.BLUE);
            robot.setHeading(90, 0.7);
            robot.alignToBoard();
            this.sleep(100);
            break;
        }
    }
}
