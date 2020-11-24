package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;

@Autonomous(name="WobbleGoal", group="Pushbot")
public class WobbleGoalTest extends LinearOpMode {

    /* Declare OpMode members. */
    HardwarePushbot         robot   = new HardwarePushbot();   // Use a Pushbot's hardware
    private ElapsedTime     runtime = new ElapsedTime();

    static final double     COUNTS_PER_MOTOR_REV    = 537.6 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 19.2 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.8;
    static final double     TURN_SPEED              = 0.6;
    private DcMotor fleftDrive;
    private DcMotor rleftDrive;
    private DcMotor frightDrive;
    private DcMotor rrightDrive;

    @Override
    public void runOpMode() {

        fleftDrive  = hardwareMap.get(DcMotor.class, "frontleft");
        frightDrive = hardwareMap.get(DcMotor.class, "frontright");
        rrightDrive = hardwareMap.get(DcMotor.class, "backright");
        rleftDrive = hardwareMap.get(DcMotor.class, "backleft");

        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        fleftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rleftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rrightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        fleftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rleftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rrightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("Path0",  "Starting at %7d :%7d",
                fleftDrive.getCurrentPosition(),
                frightDrive.getCurrentPosition());
        telemetry.update();

        waitForStart();

        encoderDrive(DRIVE_SPEED,  108,  108, 22.0);
        encoderDrive(TURN_SPEED,   -30, 30, 8.0);

        telemetry.addData("Path", "Complete");
        telemetry.update();
    }

    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newfrontLeftTarget;
        int newfrontRightTarget;
        int newbackRightTarget;
        int newbackLeftTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newfrontLeftTarget = fleftDrive.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newfrontRightTarget = frightDrive.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            newbackLeftTarget = rleftDrive.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newbackRightTarget = rrightDrive.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            fleftDrive.setTargetPosition(newfrontLeftTarget);
            frightDrive.setTargetPosition(newfrontRightTarget);
            rleftDrive.setTargetPosition(newbackLeftTarget);
            rrightDrive.setTargetPosition(newbackRightTarget);

            // Turn On RUN_TO_POSITION
            fleftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rleftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rrightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            fleftDrive.setPower(Math.abs(speed));
            frightDrive.setPower(Math.abs(speed));
            rleftDrive.setPower(Math.abs(speed));
            rrightDrive.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (fleftDrive.isBusy() && frightDrive.isBusy() && rleftDrive.isBusy() && rrightDrive.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newfrontLeftTarget,  newfrontRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        fleftDrive.getCurrentPosition(),
                        frightDrive.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            fleftDrive.setPower(0);
            frightDrive.setPower(0);
            rleftDrive.setPower(0);
            rrightDrive.setPower(0);

            // Turn off RUN_TO_POSITION
            fleftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rleftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rrightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
}
