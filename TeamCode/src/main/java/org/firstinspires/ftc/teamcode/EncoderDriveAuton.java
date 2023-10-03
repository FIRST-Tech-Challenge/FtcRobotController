package org.firstinspires.ftc.teamcode.team_19446;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous
public class EncoderAutonomous extends LinearOpMode {

    private DcMotor motorFrontLeft;
    private DcMotor motorFrontRight;
    private DcMotor motorBackLeft;
    private DcMotor motorBackRight;

    // Define constants for motor encoder counts
    private static final int TICKS_PER_REVOLUTION = 1440; // Replace with your motor's ticks per revolution
    private static final double WHEEL_DIAMETER_INCHES = 4.0; // Replace with your wheel diameter
    private static final double DRIVE_SPEED = 0.5; // Adjust the speed as needed
    private static final int TARGET_DISTANCE_INCHES = 12; // Replace with your target distance

    @Override
    public void runOpMode() {
        // Initialize your motors
        motorFrontLeft = hardwareMap.get(DcMotor.class, "motorFrontLeft");
        motorFrontRight = hardwareMap.get(DcMotor.class, "motorFrontRight");
        motorBackLeft = hardwareMap.get(DcMotor.class, "motorBackLeft");
        motorBackRight = hardwareMap.get(DcMotor.class, "motorBackRight");

        // Reverse motors if needed
        motorFrontLeft.setDirection(DcMotor.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotor.Direction.REVERSE);

        // Set motor modes to run using encoders
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        // Calculate the target encoder ticks based on distance
        int targetTicks = (int) (TICKS_PER_REVOLUTION * TARGET_DISTANCE_INCHES / (Math.PI * WHEEL_DIAMETER_INCHES));

        // Reset encoders
        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set the target position for all motors
        motorFrontLeft.setTargetPosition(targetTicks);
        motorFrontRight.setTargetPosition(targetTicks);
        motorBackLeft.setTargetPosition(targetTicks);
        motorBackRight.setTargetPosition(targetTicks);

        // Set motor power
        motorFrontLeft.setPower(DRIVE_SPEED);
        motorFrontRight.setPower(DRIVE_SPEED);
        motorBackLeft.setPower(DRIVE_SPEED);
        motorBackRight.setPower(DRIVE_SPEED);

        // Run to target position
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Wait until all motors reach the target position
        while (opModeIsActive() &&
                motorFrontLeft.isBusy() &&
                motorFrontRight.isBusy() &&
                motorBackLeft.isBusy() &&
                motorBackRight.isBusy()) {
            // You can add other actions or checks here
            // For example, use sensors to detect obstacles or perform other tasks
        }

        // Stop all motors
        motorFrontLeft.setPower(0);
        motorFrontRight.setPower(0);
        motorBackLeft.setPower(0);
        motorBackRight.setPower(0);

        // Reset motor modes to RUN_USING_ENCODER
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Your autonomous actions after reaching the target position
    }
}
