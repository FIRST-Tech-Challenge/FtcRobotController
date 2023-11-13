package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
public class TestingWade extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        Robot robot = new Robot(hardwareMap, this, telemetry, true);
        robot.setUpDrivetrainMotors();
        waitForStart();

        while (opModeIsActive()) {

            robot.mecanumBlocking(2, true, 0.75);
            this.sleep(2000);
            robot.mecanumBlocking(2, false, 0.75);
            break;
        }
    }
}
