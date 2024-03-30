package org.firstinspires.ftc.teamcode.kaitlyn;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Robot;

@TeleOp
public class kaitlynTest extends LinearOpMode {
    Robot robot;
    @Override
    public void runOpMode() throws InterruptedException {

        robot = new Robot(hardwareMap, this, telemetry, true, true, false);
        waitForStart();
        robot.setUpDrivetrainMotors();
        while (opModeIsActive()) {
            telemetry.addData("fleft", robot.fLeft.getCurrentPosition());
            telemetry.addData("3 inch to tick", robot.bRightMecanumController.convertInchesToTicks(3));
            telemetry.update();
            robot.mecanumParallel(3);
        }
    }
}
