package org.firstinspires.ftc.teamcode.Alan;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.bosch.BNO055IMU;
// the best drive ever. USE THIS!!!!!
@TeleOp
public class fieldOrientedDrive2 extends LinearOpMode {

    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private BNO055IMU imu;
    private double headingOffset = 0;

    @Override
    public void runOpMode() {
        // Initialize hardware
        frontLeft = hardwareMap.get(DcMotor.class, "fLeft");
        frontRight = hardwareMap.get(DcMotor.class, "fRight");
        backLeft = hardwareMap.get(DcMotor.class, "bLeft");
        backRight = hardwareMap.get(DcMotor.class, "bRight");

        // Initialize IMU
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        imu.initialize(parameters);

        // Wait for the game to start
        waitForStart();

        while (opModeIsActive()) {
            // Get current heading
            double heading = imu.getAngularOrientation().firstAngle;
            double headingInRadians = Math.toRadians(heading);

            // Get gamepad input
            double drive = -gamepad1.left_stick_y;  // Forward/backward movement
            double strafe = gamepad1.left_stick_x;  // Sideways movement
            double rotate = gamepad1.right_stick_x; // Rotation

            // Calculate field-oriented input
            double temp = drive * Math.cos(headingInRadians) + strafe * Math.sin(headingInRadians);
            strafe = -drive * Math.sin(headingInRadians) + strafe * Math.cos(headingInRadians);
            drive = temp;

            // Calculate motor power
            double frontLeftPower = drive + strafe + rotate;
            double frontRightPower = drive - strafe - rotate;
            double backLeftPower = drive - strafe + rotate;
            double backRightPower = drive + strafe - rotate;

            // Normalize motor powers
            double maxPower = Math.max(Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower)),
                    Math.max(Math.abs(backLeftPower), Math.abs(backRightPower)));
            if (maxPower > 1.0) {
                frontLeftPower /= maxPower;
                frontRightPower /= maxPower;
                backLeftPower /= maxPower;
                backRightPower /= maxPower;
            }

            // Set motor powers
            frontLeft.setPower(frontLeftPower);
            frontRight.setPower(frontRightPower);
            backLeft.setPower(backLeftPower);
            backRight.setPower(backRightPower);
        }
    }
}
