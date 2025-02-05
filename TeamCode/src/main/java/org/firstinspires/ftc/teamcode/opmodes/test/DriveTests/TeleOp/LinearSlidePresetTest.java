package org.firstinspires.ftc.teamcode.opmodes.test.DriveTests.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

@TeleOp(name="Linear Slide Presets")
public class LinearSlidePresetTest extends LinearOpMode {
    private DcMotor slideMotor;

    // Define preset positions (in encoder ticks)
    private final int GROUND_POSITION = 0;
    private final int LOW_POSITION = -500;
    private final int MID_POSITION = -1000;
    private final int HIGH_POSITION = -1500;

    @Override
    public void runOpMode() {
        slideMotor = hardwareMap.get(DcMotor.class, "armMotor");
        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.a) {
                moveToPosition(GROUND_POSITION);
            } else if (gamepad1.b) {
                moveToPosition(LOW_POSITION);
            } else if (gamepad1.x) {
                moveToPosition(MID_POSITION);
            } else if (gamepad1.y) {
                moveToPosition(HIGH_POSITION);
            } else if (gamepad2.dpad_right) {
                moveToPosition(MID_POSITION);
                // Additional code for adjusting the linear slide arm angle and claw position goes here
            }

            telemetry.addData("Current Position", slideMotor.getCurrentPosition());

            telemetry.update();
        }
    }

    private void moveToPosition(int targetPosition) {
        slideMotor.setTargetPosition(targetPosition);
        slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideMotor.setPower(1.0);

        while (slideMotor.isBusy() && opModeIsActive()) {
            telemetry.addData("Moving to", targetPosition);
            telemetry.addData("Current Position", slideMotor.getCurrentPosition());
            telemetry.update();
        }

        slideMotor.setPower(0);
        slideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}


