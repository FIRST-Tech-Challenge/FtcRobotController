package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
public class ShortRedAuto extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        //robot, dt motors, vision processing setup
        Robot robot = new Robot(hardwareMap, this, telemetry, true, true);
        robot.setUpDrivetrainMotors();
        robot.initVisionProcessing();

        waitForStart();

        while (opModeIsActive()) {
            robot.detectMarkerPosition();
            robot.shortMoveToBoard();
            robot.alignToBoard();
            robot.autoOuttake(true);
            robot.parkBot(false);
            break;
        }
    }
}

