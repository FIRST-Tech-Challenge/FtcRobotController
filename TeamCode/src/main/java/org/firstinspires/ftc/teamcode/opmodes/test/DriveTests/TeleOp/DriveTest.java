package org.firstinspires.ftc.teamcode.opmodes.test.DriveTests.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

//@TeleOp(name = "GoBilda Drive Control", group = "Linear OpMode")
public class DriveTest extends LinearOpMode {

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
        // Set motor directions: Reverse the motors on one side to ensure correct movement
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

        // Initialize IMU hardware and parameters
        IMU imu = hardwareMap.get(IMU.class, "imu");

        imu = (IMU) hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters imuParameters = new BNO055IMU.Parameters();
        imuParameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;

        // Display initialization status on Driver Station
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses START)
        waitForStart();
        runtime.reset();

        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Get input from the joysticks
            double y = -gamepad1.left_stick_y;   // forward/backward
            double x = gamepad1.left_stick_x;    // strafe (left/right)
            double rx = gamepad1.right_stick_x;  // rotation (turn)

            // Reset IMU yaw if the options button is pressed
            if (gamepad1.options) {
            }

            // Retrieve the robot heading in radians from the IMU
            double botHeading = ((BNO055IMU) imu).getAngularOrientation().firstAngle;

            // Adjust for robot's orientation (field-centric control)
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            // Optional strafe power adjustment for better control
            rotX *= 1.1;

            // Determine the largest motor power needed for normalization
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;

            // Set power to each motor
            leftFrontDrive.setPower(frontLeftPower);
            leftBackDrive.setPower(backLeftPower);
            rightFrontDrive.setPower(frontRightPower);
            rightBackDrive.setPower(backRightPower);

            // Display telemetry information
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "Front L/R: (%.2f, %.2f), Back L/R: (%.2f, %.2f)",
                    frontLeftPower, frontRightPower, backLeftPower, backRightPower);
            telemetry.addData("Heading", botHeading);
            telemetry.update();
        }
    }
}
