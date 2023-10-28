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
            robot.straightBlocking(24, true);
            sleep(100);
            robot.setHeading(0);
            sleep(1000);
            robot.setHeading(90);
            sleep(1000);
            break;
        }
    }
}
