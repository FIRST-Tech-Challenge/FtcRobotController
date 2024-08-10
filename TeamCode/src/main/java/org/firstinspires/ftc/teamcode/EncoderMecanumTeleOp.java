package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class EncoderMecanumTeleOp extends OpMode{
    /*
     * The mecanum drivetrain involves four separate motors that spin in
     * different directions and different speeds to produce the desired
     * movement at the desired speed.
     */

    // declare and initialize four DcMotors.
    private DcMotor front_left  = null;
    private DcMotor front_right = null;
    private DcMotor back_left   = null;
    private DcMotor back_right  = null;

    final double HD_COUNTS_PER_REV = 28;
    final double DRIVE_GEAR_REDUCTION = 20.15293;
    final double WHEEL_CIRCUMFERENCE_MM = 96 * Math.PI;
    final double DRIVE_COUNTS_PER_MM = (HD_COUNTS_PER_REV * DRIVE_GEAR_REDUCTION) / WHEEL_CIRCUMFERENCE_MM;
    final double DRIVE_COUNTS_PER_IN = DRIVE_COUNTS_PER_MM * 25.4;

    @Override
    public void init() {

        // Name strings must match up with the config on the Robot Controller
        // app.
        front_left   = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        front_right  = hardwareMap.get(DcMotor.class, "frontRightMotor");
        back_left	 = hardwareMap.get(DcMotor.class, "backLeftMotor");
        back_right   = hardwareMap.get(DcMotor.class, "backRightMotor");

        front_right.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void loop() {
        // DRIVE CODE
        double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
        double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
        double rx = gamepad1.right_stick_x;

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        final int encoderMultiplier = 5;
        int front_left_target_IN = (int) (((y + x + rx) / denominator) * encoderMultiplier);
        int back_left_target_IN = (int) ((y - x + rx) / denominator * encoderMultiplier);
        int front_right_target_IN = (int) ((y - x - rx) / denominator * encoderMultiplier);
        int back_right_target_IN = (int) ((y + x - rx) / denominator * encoderMultiplier);

        front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        int front_right_target = (int)(front_right_target_IN * DRIVE_COUNTS_PER_IN);
        int front_left_target = (int)(front_left_target_IN * DRIVE_COUNTS_PER_IN);
        int back_right_target = (int)(back_right_target_IN * DRIVE_COUNTS_PER_IN);
        int back_left_target = (int)(back_left_target_IN * DRIVE_COUNTS_PER_IN);

        front_right.setTargetPosition(front_right_target);
        front_left.setTargetPosition(front_left_target);
        back_right.setTargetPosition(back_right_target);
        back_left.setTargetPosition(back_left_target);

        front_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        front_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        back_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        back_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        front_right.setPower(1);
        front_left.setPower(1);
        back_right.setPower(1);
        back_left.setPower(1);

        /*
        while (front_right.isBusy() || front_left.isBusy() || back_right.isBusy() || back_left.isBusy()) {
            telemetry.addLine("Moving!");
            telemetry.update();
        }

        front_right.setPower(0);
        front_left.setPower(0);
        back_right.setPower(0);
        back_left.setPower(0);
         */

        telemetry.addData("Left Stick Y", y);
        telemetry.addData("Left Stick X", x);
        telemetry.addData("Right Stick X", rx);
        telemetry.update();
    }
}
