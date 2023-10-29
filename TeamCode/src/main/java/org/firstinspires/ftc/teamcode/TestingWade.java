package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
public class TestingWade extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        Robot robot = new Robot(hardwareMap, this, telemetry);
        robot.setUpDrivetrainMotors();
        waitForStart();

        while (opModeIsActive()) {
            robot.straightBlocking(24, true, 0.75);
            sleep(100);
            robot.setHeading(0, 1);
            sleep(1000);
            robot.setHeading(90, 1);
            sleep(1000);
            break;
        }
    }
}
