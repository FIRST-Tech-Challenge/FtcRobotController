import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous
public class EncoderDriveAutonomous extends LinearOpMode {

    private DcMotor motorFrontLeft;
    private DcMotor motorFrontRight;
    private DcMotor motorBackLeft;
    private DcMotor motorBackRight;

    // Constants for PID control
    private static final double kPDrive = 0.68;
    private static final double kIDrive = 0.005;
    private static final double kDDrive = 0.0;
    private static final double KpTurnCorrection = 2.5;
    private static final double moveError = 1.5;

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

        waitForStart();

        // Variables for tracking progress
        int endCount = 0;

        // Variables for PID control
        double errorDrive = targetEncoder;
        double integralDrive = 0.0;
        double derivativeDrive = 0.0;
        double prevErrorDrive = 0.0;

        // Reset the drive motor encoders
        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Main PID control loop
        while (endCount < 2 && opModeIsActive()) {
            // Get the current encoder counts for the left and right drive motors
            int leftEncoderValue = motorFrontLeft.getCurrentPosition();
            int rightEncoderValue = motorFrontRight.getCurrentPosition();
            if (Math.abs(errorDrive) < moveError) {
                endCount += 1;
            } else {
                endCount = 0;
            }

            // Calculate the distance traveled by the robot
            double currentDist = (leftEncoderValue + rightEncoderValue) / 2.0;

            // Calculate the error, integral, and derivative terms for PID control
            prevErrorDrive = errorDrive;
            errorDrive = targetEncoder - currentDist;
            if (prevErrorDrive * errorDrive < 0) {
                integralDrive = 0;
            }
            if (Math.abs(errorDrive) < 50) {
                integralDrive += errorDrive;
            }
            derivativeDrive = errorDrive - prevErrorDrive;

            // Calculate the motor speeds using PID control
            double leftSpeed = kPDrive * errorDrive + kIDrive * integralDrive + kDDrive * derivativeDrive;
            double rightSpeed = kPDrive * errorDrive + kIDrive * integralDrive + kDDrive * derivativeDrive;

            // Limit the motor speeds to be within the acceptable range
            leftSpeed = Range.clip(leftSpeed, -kMaxSpeed, kMaxSpeed);
            rightSpeed = Range.clip(rightSpeed, -kMaxSpeed, kMaxSpeed);

            leftSpeed += KpTurnCorrection * (heading - imu.getHeading());
            rightSpeed -= KpTurnCorrection * (heading - imu.getHeading());

            // Set the motor speeds
            motorFrontLeft.setPower(leftSpeed);
            motorFrontRight.setPower(rightSpeed);
            motorBackLeft.setPower(leftSpeed);
            motorBackRight.setPower(rightSpeed);

            // Wait for the motors to update
            sleep(20);
        }

        // Stop the motors once the target distance has been reached
        motorFrontLeft.setPower(0);
        motorFrontRight.setPower(0);
        motorBackLeft.setPower(0);
        motorBackRight.setPower(0);
    }
}
