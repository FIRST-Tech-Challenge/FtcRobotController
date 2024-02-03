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

        Robot robot = new Robot(hardwareMap, this, telemetry, true, false, true);
        robot.setUpDrivetrainMotors();
        robot.setUpIntakeOuttake();

        waitForStart();

        while (opModeIsActive()) {

            this.sleep(2000);
            robot.stackAttachmentOut();
            robot.intake.setPower(-1);
            robot.openClamp(true, true);
            this.sleep(300);
            robot.straightBlocking(23, true, 0.3);
            this.sleep(300);
            robot.straightBlocking(3, false, 0.5);
            this.sleep(300);
            robot.straightBlocking(3, true, 0.5);
            this.sleep(300);
            robot.closeClamp(true);
            robot.straightBlocking(3, false, 0.5);
            this.sleep(300);
            robot.intake.setPower(1);
            robot.straightBlocking2(-20);

            break;

            //go, come back, go, come back, clamp, regurgitate

        }
    }
}
