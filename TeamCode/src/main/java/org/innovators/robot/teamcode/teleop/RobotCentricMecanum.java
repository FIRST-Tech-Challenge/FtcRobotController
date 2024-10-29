package org.innovators.robot.teamcode.teleop;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.innovators.robot.teamcode.hardware.RobotHardware;
import org.innovators.robot.teamcode.util.Constants;

@TeleOp(name="Field Centric Mecanum", group="TeleOp")
public class RobotCentricMecanum extends LinearOpMode {
//    private DcMotor frontLeftDrive = null;
//    private DcMotor frontRightDrive = null;
//    private DcMotor backLeftDrive = null;
//    private DcMotor backRightDrive = null;
//    private BNO055IMU imu;
//    private Orientation angles;
    private final RobotHardware robot = new RobotHardware();
    private final ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

        // Reset the runtime
        runtime.reset();

        // Initialize motors
        DcMotor frontLeftDrive = hardwareMap.get(DcMotor.class, "front_left_drive");
        DcMotor frontRightDrive = hardwareMap.get(DcMotor.class, "front_right_drive");
        DcMotor backLeftDrive = hardwareMap.get(DcMotor.class, "back_left_drive");
        DcMotor backRightDrive = hardwareMap.get(DcMotor.class, "back_right_drive");

        frontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.REVERSE);

        // Initialize IMU
        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        imu.initialize(parameters);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            // Get joystick values
            double y = -gamepad1.left_stick_y * Constants.DRIVE_SPEED; // Forward/Backward // Inverted Y axis
            double x = gamepad1.left_stick_x * Constants.DRIVE_SPEED; // Left/Right
            double rx = gamepad1.right_stick_x * Constants.TURN_SPEED; // Rotation

            // Get robot orientation
            Orientation angles = imu.getAngularOrientation();
            double botHeading = -angles.firstAngle;

            // Calculate field-centric values
            double rotX = x * Math.cos(botHeading) - y * Math.sin(botHeading);
            double rotY = x * Math.sin(botHeading) + y * Math.cos(botHeading);

            // Calculate motor powers
            double frontLeftPower = rotY + rotX + rx;
            double frontRightPower = rotY - rotX - rx;
            double backLeftPower = rotY - rotX + rx;
            double backRightPower = rotY + rotX - rx;

            // Set motor powers
            frontLeftDrive.setPower(Range.clip(frontLeftPower, -1.0, 1.0));
            frontRightDrive.setPower(Range.clip(frontRightPower, -1.0, 1.0));
            backLeftDrive.setPower(Range.clip(backLeftPower, -1.0, 1.0));
            backRightDrive.setPower(Range.clip(backRightPower, -1.0, 1.0));

            // Telemetry for debugging
            telemetry.addData("Status", "Run Time: " + runtime);
            telemetry.addData("Heading", botHeading);
            telemetry.update();
        }
    }
}
