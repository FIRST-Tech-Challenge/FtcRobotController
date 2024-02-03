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

            robot.stackAttachmentOut();
            robot.intake.setPower(-1);
            robot.openClamp(true, true);
            /*
            this.sleep(2000);
            robot.straightBlocking(20, true, 0.5);
            sleep(2000);
            break;
            */

            //go, come back, go, come back, clamp, regurgitate

        }
    }
}
