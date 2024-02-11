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

            robot.openClamp(true, true, true);
            robot.intake.setPower(-1);
            robot.straightBlocking(5, true, 1);
            robot.straightBlocking(1, false, 1);
            robot.straightBlocking(1, true, 1);
            robot.straightBlocking(1, false, 1);
            robot.straightBlocking(1, true, 1);
            robot.straightBlocking(1, false, 1);
            robot.straightBlocking(1, true, 1);
            robot.straightBlocking(1, false, 1);
            robot.mecanumBlocking2(1);
            robot.straightBlocking(1, true, 1);
            robot.straightBlocking(1, false, 1);
            robot.straightBlocking(1, true, 1);
            robot.straightBlocking(1, false, 1);
            robot.straightBlocking(1, true, 1);
            robot.straightBlocking(1, false, 1);
            robot.closeClamp(true);

            break;
        }
    }
}
