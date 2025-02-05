package org.firstinspires.ftc.teamcode.opmodes.test.DriveTests;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

@TeleOp(name = "TEST TELEOP", group = "Linear OpMode")
public class TeleopTest extends LinearOpMode {

    // Declare OpMode members for the 4 motors, IMU, and elapsed time
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private BNO055IMU imu;



    @Override
    public void runOpMode() {

        // Initialize the hardware variables for the 4 mecanum motors
        leftFrontDrive = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        leftBackDrive = hardwareMap.get(DcMotor.class, "backLeftMotor");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "frontRightMotor");
        rightBackDrive = hardwareMap.get(DcMotor.class, "backRightMotor");

        // Set motor directions: Reverse the motors on one side to ensure correct movement
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
       // rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
       // rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

        // Initialize IMU hardware and parameters
       // imu = hardwareMap.get(BHI260AP.class, "imu");
       // BHI260AP .Parameters imuParameters = new BNO055IMU.Parameters();
        //imuParameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
      //  imu.initialize(imuParameters);


        // Display initialization status on Driver Station
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses START)
        waitForStart();
        runtime.reset();

        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            double y = Math.pow(-gamepad1.left_stick_y,3); // Remember, Y stick value is reversed
            double x = Math.pow(gamepad1.left_stick_x * 1.1,3); // Counteract imperfect strafing
            double rx = Math.pow(gamepad1.right_stick_x,3);

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 2);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            leftFrontDrive.setPower(frontLeftPower);
            leftBackDrive.setPower(backLeftPower);
            rightFrontDrive.setPower(frontRightPower);
            rightBackDrive.setPower(backRightPower);




            // Display telemetry information
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "Front L/R: (%.2f, %.2f), Back L/R: (%.2f, %.2f)",
                    frontLeftPower, frontRightPower, backLeftPower, backRightPower);
            telemetry.update();

        }
    }
}
