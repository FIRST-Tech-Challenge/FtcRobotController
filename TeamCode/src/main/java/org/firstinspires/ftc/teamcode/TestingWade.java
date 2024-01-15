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

        Robot robot = new Robot(hardwareMap, this, telemetry, false, true);
        robot.setUpDrivetrainMotors();
        robot.setUpIntakeOuttake();

        waitForStart();

        while (opModeIsActive()) {
            /*
            robot.stackAttachment.setPosition(0.5);
            robot.trayAngle.setPosition(0.5);
            */

            robot.mecanumBlocking2(36); // right
            this.sleep(2000);
            robot.mecanumBlocking2(-36); // left
            this.sleep(2000);

            /*
            robot.mecanumBlockingFixHeading(-24); //2 motors
            this.sleep(2000);
            robot.mecanumBlockingFixHeading(24);
            this.sleep(2000);
            */

            //robot.mecanumBlockingTwoMotors(-24);
            //this.sleep(2000);
            //robot.mecanumBlockingTwoMotors(24);
            //this.sleep(2000);

            /*
            robot.setHeading2(90);
            this.sleep(2000);
            robot.setHeading2(0);
            this.sleep(2000);
            */

            break;
        }
    }
}
