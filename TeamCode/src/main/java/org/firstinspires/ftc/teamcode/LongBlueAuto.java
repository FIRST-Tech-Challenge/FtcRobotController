package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
public class LongBlueAuto extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        //robot, dt motors, vision processing setup
        Robot robot = new Robot(hardwareMap, this, telemetry, false);
        robot.setUpDrivetrainMotors();
        robot.initVisionProcessing(MarkerDetector.ALLIANCE_COLOR.BLUE);

        waitForStart();

        //TODO: unlock slide, clamp down, slide down (not nec in that order)

        robot.detectMarkerPosition();
        robot.longMoveToBoard();
        robot.alignToBoard();
        robot.autoOuttake();
    }
}

