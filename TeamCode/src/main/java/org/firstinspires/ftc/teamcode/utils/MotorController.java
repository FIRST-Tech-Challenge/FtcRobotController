package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.R;

@Disabled
@Deprecated
public class MotorController {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();

    public static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    public static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;     // This is < 1.0 if geared UP
    public static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    public static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
    public static final double     DRIVE_SPEED             = 1;
    public static final double     TURN_SPEED              = 0.5;

    private Telemetry telemetry;
    private HardwareMap hardwareMap;
    private LinearOpMode linearOpMode;

    DcMotor leftDrive;
    DcMotor rightDrive;
    DcMotor centerDrive;

    int ld1Offset;
    int rd1Offset;
    int cd1Offset;

    /**
     * Temporary abstracted motor controller for the HDrive drivetrain
     * @param tele The telemetry of the robot
     * @param hardware The hardware of the robot
     * @param opMode The Linear opMode to associate with this controller
     * */
    public MotorController(Telemetry tele, HardwareMap hardware, LinearOpMode opMode) {
        telemetry = tele;
        hardwareMap = hardware;
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
        centerDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0",  "Starting at   ",
                leftDrive.getCurrentPosition(),
                rightDrive.getCurrentPosition(),
                centerDrive.getCurrentPosition());
        telemetry.update();
    }

    /**
     * Moves the robot by one leg of a path
     * @param speed The speed of the robot
     * @param leftInches The distance to move the left motor
     * @param rightInches The distance to move the right motor
     * @param centerInches The distance to move the center motor
     * @param timeoutSeconds The amount of time before the robot cancels the motion
     * @param pathLeg The name of the leg being driven
     * */
    public void drive(double speed, double leftInches, double rightInches, double centerInches, double timeoutSeconds, String pathLeg) {
        doTheDriving(speed, leftInches, rightInches, centerInches, timeoutSeconds);
        telemetry.addData("Path", "Leg " + pathLeg + "complete");
        telemetry.update();
    }

    private void doTheDriving(double speed, double leftInches, double rightInches, double centerInches, double timeoutS) {
        int newLeftTarget;
        int newRightTarget;
        int newCenterTarget;

        // Ensure that the opmode is still active
        if (linearOpMode.opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = leftDrive.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH * ld1Offset);
            newRightTarget = rightDrive.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH * rd1Offset);
            newCenterTarget = centerDrive.getCurrentPosition() + (int)(centerInches * COUNTS_PER_INCH * cd1Offset);
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

            while (linearOpMode.opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (leftDrive.isBusy() || rightDrive.isBusy() || centerDrive.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to   ", newLeftTarget,  newRightTarget, newCenterTarget);
                telemetry.addData("Path2",  "Running at   ",
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

            linearOpMode.sleep(2000);   // optional pause after each move
        }
    }

}
