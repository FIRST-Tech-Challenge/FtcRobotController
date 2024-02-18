package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
public class TestingWade extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        Robot robot = new Robot(hardwareMap, this, telemetry, true, false, true);
        robot.setUpDrivetrainMotors();
        robot.setUpIntakeOuttake();

        waitForStart();

        while (opModeIsActive()) {

            robot.setServoPos(robot.spikeServo, 0.68); //closed 0.5 open 0.2
            // open 0.5 close 0.68

        }
    }
}
