package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
public class LongRedAuto extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        //robot, dt motors, vision processing setup
        Robot robot = new Robot(hardwareMap, this, telemetry, true);
        robot.setUpDrivetrainMotors();
        robot.initVisionProcessingRed();

        waitForStart();

        //TODO: unlock slide, clamp down, slide down (not nec in that order)

        robot.detectMarkerPositionRed();
        robot.moveToBoard();
        robot.alignToBoard();
        robot.autoOuttake();
    }
}

