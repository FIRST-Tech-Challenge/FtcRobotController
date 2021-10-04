package org.firstinspires.ftc.teamcode.testing.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;
import org.firstinspires.ftc.teamcode.R;

@Autonomous(name="Encoder Drive Test (H-Drive)", group="H.Testing.Autonomous")
public class AutoEncoderDriveTest extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime     runtime = new ElapsedTime();

    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                                      (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;

    DcMotor leftDrive;
    DcMotor rightDrive;
    DcMotor centerDrive;

    int ld1Offset;
    int rd1Offset;
    int cd1Offset;

    @Override
    public void runOpMode() {
        leftDrive = hardwareMap.get(DcMotor.class, hardwareMap.appContext.getString(R.string.LEFT_DRIVE_1));
        rightDrive = hardwareMap.get(DcMotor.class, hardwareMap.appContext.getString(R.string.RIGHT_DRIVE_1));
        centerDrive = hardwareMap.get(DcMotor.class, hardwareMap.appContext.getString(R.string.CNTR_DRIVE_1));

        ld1Offset = hardwareMap.appContext.getResources().getInteger(R.integer.ld1_offset);
        rd1Offset = hardwareMap.appContext.getResources().getInteger(R.integer.rd1_offset);
        cd1Offset = hardwareMap.appContext.getResources().getInteger(R.integer.cd1_offset);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        centerDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        centerDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0",  "Starting at %7d : %7d : %7d",
                            leftDrive.getCurrentPosition(),
                            rightDrive.getCurrentPosition(),
                            centerDrive.getCurrentPosition());
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        encoderDrive(DRIVE_SPEED,  12,  12, 0, 5.0);
        encoderDrive(TURN_SPEED,   6, -6, 0, 4.0);
        encoderDrive(DRIVE_SPEED, -6, -6, 0, 4.0);
        encoderDrive(DRIVE_SPEED, 0, 0, 5, 4.0);
        encoderDrive(DRIVE_SPEED, 0, 0, -5, 4.0);

        telemetry.addData("Path", "Complete");
        telemetry.update();
    }

    /*
     *  Method to perform a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    public void encoderDrive(double speed,
                             double leftInches, double rightInches, double centerInches,
                             double timeoutS) {
        int newLeftTarget;
        int newRightTarget;
        int newCenterTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = leftDrive.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightTarget = rightDrive.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            newCenterTarget = centerDrive.getCurrentPosition() + (int)(centerInches * COUNTS_PER_INCH);
            leftDrive.setTargetPosition(newLeftTarget);
            rightDrive.setTargetPosition(newRightTarget);
            centerDrive.setTargetPosition(newCenterTarget);

            // Turn On RUN_TO_POSITION
            leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            centerDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            leftDrive.setPower(Math.abs(speed));
            rightDrive.setPower(Math.abs(speed));
            centerDrive.setPower(Math.abs(speed));

            while (opModeIsActive() &&
                   (runtime.seconds() < timeoutS) &&
                   (leftDrive.isBusy() && rightDrive.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d : %7d : %7d", newLeftTarget,  newRightTarget, newCenterTarget);
                telemetry.addData("Path2",  "Running at %7d : %7d : %7d",
                                            leftDrive.getCurrentPosition(),
                                            rightDrive.getCurrentPosition(),
                                            centerDrive.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            leftDrive.setPower(0);
            rightDrive.setPower(0);
            centerDrive.setPower(0);

            // Turn off RUN_TO_POSITION
            leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            centerDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(2000);   // optional pause after each move
        }
    }
}
