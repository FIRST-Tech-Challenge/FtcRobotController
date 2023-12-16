package org.firstinspires.ftc.teamcode.kaitlyn;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;

public class TestAuto extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        //robot, dt motors, vision processing setup
        Robot robot = new Robot(hardwareMap, this, telemetry, false, true);
        robot.setUpDrivetrainMotors();
        robot.initVisionProcessing();

        waitForStart();

        if (opModeIsActive()) {
            robot.detectMarkerPosition();
            //robot.longMoveToBoard();
            //robot.alignToBoard();
            //robot.autoOuttake();
            //robot.parkLeft();
        }
    }
}

