package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
public class LongRedAuto extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        //robot, dt motors, vision processing setup
        Robot robot = new Robot(hardwareMap, this, telemetry, true);
        robot.setUpDrivetrainMotors();
        robot.initVisionProcessing(MarkerDetector.ALLIANCE_COLOR.RED);

        waitForStart();

        robot.detectMarkerPosition();
        robot.longMoveToBoard();
        robot.alignToBoard();
        robot.autoOuttake();

        //left mecanum 20
        //center mecanum 26
        //right mecanum 32

        //TODO: write this for shortauto (parkright)
    }
}

