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
        double slideStartingPosition;


        //double trayAngle = 0.52;

        waitForStart();
        //robot.trayAngle.setPosition(trayAngle);
        //robot.trayToOuttakePos(false);


        while (opModeIsActive()) {
            slideStartingPosition = robot.lsFront.getCurrentPosition() + 50; //fake zero = 50 so slides don't slam down

            // move linear slide up
            robot.trayToIntakePos(true);
            robot.moveLinearSlideByTicksBlocking(2000 + slideStartingPosition);
            sleep(2000);
            robot.trayToOuttakePos(true);
            sleep(2000);
            break;
        }
    }
}
