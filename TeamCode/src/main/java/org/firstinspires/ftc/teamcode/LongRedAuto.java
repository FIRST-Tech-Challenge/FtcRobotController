package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
public class LongRedAuto extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        //robot, dt motors, vision processing setup
        Robot robot = new Robot(hardwareMap, this, telemetry, true, true);
        robot.setUpDrivetrainMotors();
        robot.initVisionProcessing();
        waitForStart();

        while (opModeIsActive()) {
            robot.detectMarkerPosition();
            robot.longMoveToBoard();
            //robot.alignToBoard();
            //robot.autoOuttake();
            //robot.parkBot(true);
            break;
        }
    }
}

