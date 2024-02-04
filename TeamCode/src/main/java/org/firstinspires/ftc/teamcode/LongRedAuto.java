package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
public class LongRedAuto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        //robot, dt motors, vision processing setup
        Robot robot = new Robot(hardwareMap, this, telemetry, true, true, true);
        robot.setUpDrivetrainMotors();
        robot.setUpIntakeOuttake();
        robot.initVisionProcessing();
        double slideStartingPosition;

        waitForStart();

        while (opModeIsActive()) {
            robot.detectMarkerPosition();
            robot.longMoveToBoard(false);

            robot.closeClamp(false);
            robot.openHook();
            robot.setServoPosBlocking(robot.spikeServo, 0.5);
            sleep(100);

            robot.alignToBoardFast();

            // move slides up
            slideStartingPosition = robot.lsFront.getCurrentPosition(); //fake zero = ??? so slides don't slam down

            robot.trayToOuttakePos(true); // pivot tray to outtake position
            robot.autoOuttake(true, slideStartingPosition);

            robot.boardToMiddle();
            robot.middleToStack();
            robot.hailMaryyyyyy();
            robot.alignToBoardFast();

            // move slides up
            slideStartingPosition = robot.lsFront.getCurrentPosition(); //fake zero = ??? so slides don't slam down

            robot.trayToOuttakePos(true); // pivot tray to outtake position
            robot.autoOuttake(false, slideStartingPosition);

            break;

        }
    }
}

// todo write timeout for apriltag final forward
// todo how to stop streaming
// todo bring back to board
// todo set complementary tag id
// todo slide not high enough second time
