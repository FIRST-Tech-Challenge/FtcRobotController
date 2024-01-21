package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous
public class TestingWadeTwo extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        Robot robot = new Robot(hardwareMap, this, telemetry, false, true);
        robot.setUpDrivetrainMotors();
        robot.setUpIntakeOuttake();

        double trayAngle = 0.52;

        waitForStart();
        //robot.trayAngle.setPosition(trayAngle);
        //robot.trayToOuttakePos(false);


        while (opModeIsActive()) {
            robot.straightBlocking2(48);

            break;
        }
    }
}
