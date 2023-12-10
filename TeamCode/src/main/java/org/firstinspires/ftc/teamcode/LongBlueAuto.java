package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
public class LongBlueAuto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        //robot, dt motors, vision processing setup
        Robot robot = new Robot(hardwareMap, this, telemetry, false, true);
        robot.setUpDrivetrainMotors();
        robot.initVisionProcessing();

        waitForStart();

        while (opModeIsActive()) {
            robot.detectMarkerPosition();
            robot.longMoveToBoard();

            // move linear slide up
            robot.moveLinearSlideByTicksBlocking(-1550);

            robot.trayToOuttakePos(true); // pivot tray to outtake position

            robot.alignToBoard();
            robot.autoOuttake(false);
            robot.parkBot(true);

            break;
        }
    }
}

