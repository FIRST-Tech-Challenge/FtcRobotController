package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
public class ShortBlueAuto extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        //robot, dt motors, vision processing setup
        Robot robot = new Robot(hardwareMap, this, telemetry, false, true);
        robot.setUpDrivetrainMotors();
        robot.initVisionProcessing();
        waitForStart();

        while (opModeIsActive()) {
            robot.detectMarkerPosition();
            robot.shortMoveToBoard();

            // move linear slide up
            robot.moveLinearSlideByTicksBlocking(-1550);

            robot.trayToOuttakePos(true); // pivot tray to outtake position

            robot.alignToBoard();
            robot.autoOuttake(true);
            robot.parkBot(false);

            /*
            //second trip
            robot.mecanumBlocking(24, true, 0.7); //mecanum directly in front of board left if blue
            robot.setHeading(-90, 0.7);
            robot.straightBlockingFixHeading(97, true, 0.8);
            robot.setHeading(-90, 0.7);

            this.sleep(100);

            robot.straightBlockingFixHeading(94, false, 0.8);
            robot.setHeading(-90, 0.7);
            robot.mecanumBlocking(24, false, 0.7); //mecanum directly in front of board left if blue
            robot.setHeading(-90, 0.7);
            */

            break;
        }
    }
}

