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

            robot.setServoPosBlocking(robot.spikeServo, 0);
            sleep(10000);
            break;
        }
    }
}
