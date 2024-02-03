package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous
public class TestingWade extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        Robot robot = new Robot(hardwareMap, this, telemetry, true, false, true);
        robot.setUpDrivetrainMotors();
        robot.setUpIntakeOuttake();

        waitForStart();

        while (opModeIsActive()) {

            // P1: (118, 35)
            robot.setHeading(0, 0.7);
            robot.mecanumBlocking2(24);
            // P2: (118, 59)
            robot.setHeading(0, 0.7);
            robot.stackAttachmentOut();
            robot.intake.setPower(-1);
            robot.straightBlocking2(95);
            // P3: (23, 59)
            robot.setHeading(0, 0.7);

            // TARGET: (60, 21)

            break;
        }
    }
}
