package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class Test extends LinearOpMode {
    private DcMotor
            frontLeftDrive,
            frontRightDrive,
            backLeftDrive,
            backRightDrive,
            armMotor;
    private final int BOTTOM = 0;
    private final int MIDDLE = 250;
    private final int TOP = 500;

    @Override
    public void runOpMode() {
        frontLeftDrive = hardwareMap.dcMotor.get("frontLeftDrive");
        frontRightDrive = hardwareMap.dcMotor.get("frontRightDrive");
        backLeftDrive = hardwareMap.dcMotor.get("backLeftDrive");
        backRightDrive = hardwareMap.dcMotor.get("backRightDrive");

        armMotor = hardwareMap.dcMotor.get("armMotor");

        frontLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        while (opModeIsActive()) {
            // Values can be customized
            double drive = gamepad1.left_stick_y * 0.5;
            double strafe = -gamepad1.left_stick_x * 0.7;
            double spin = gamepad1.right_stick_x * 0.4;

            frontLeftDrive.setPower(drive + strafe - spin);
            frontRightDrive.setPower(drive - strafe + spin);
            backLeftDrive.setPower(drive - strafe - spin);
            backRightDrive.setPower(drive + strafe + spin);

            if (gamepad1.dpad_up) {
                armMotor.setPower(0.5);
            } else if (gamepad1.dpad_down) {
                armMotor.setPower(-0.5);
            } else {
                armMotor.setPower(0);
            }
            if (gamepad1.a) {
                armEncoderMovement(0.5, TOP, true);
            } else if (gamepad1.b) {
                armEncoderMovement(0.5, BOTTOM, true);
            }
        }
    }
    public void armEncoderMovement(double power, int targetPosition, boolean logs) {
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotor.setPower(power);
        armMotor.setTargetPosition(targetPosition);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (armMotor.isBusy()) {
            if (logs) {
                telemetry.addData("Target Pos", armMotor.getTargetPosition());
                telemetry.addData("Current Pos", armMotor.getCurrentPosition());
                telemetry.update();
            }
        }
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}
