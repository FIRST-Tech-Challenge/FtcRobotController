package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.lib.Drivetrain;

@Autonomous
public class DriveAuto extends LinearOpMode {
    Drivetrain drive;
    double COUNTS_PER_INCH = 23.963313188;
    ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new Drivetrain(hardwareMap);

        waitForStart();

        encoderDrive(0.3, 12, 12, 5);
    }

    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the OpMode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = (int)(drive.get_left_position() + (leftInches * COUNTS_PER_INCH));
            newRightTarget = (int)(drive.get_right_position() + (rightInches * COUNTS_PER_INCH));
            drive.set_left_target_position(newLeftTarget);
            drive.set_right_target_position(newRightTarget);

            // Turn On RUN_TO_POSITION
            drive.set_left_mode(DcMotor.RunMode.RUN_TO_POSITION);
            drive.set_right_mode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            drive.set_left_power(Math.abs(speed));
            drive.set_right_power(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (drive.left_is_busy() && drive.right_is_busy())) {

                // Display it for the driver.
                telemetry.addData("Running to",  " %7d :%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Currently at",  " at %7d :%7d",
                        (int) drive.get_left_position(), (int) drive.get_right_position());
                telemetry.update();
            }

            // Stop all motion;
            drive.set_power(0, 0);

            // Turn off RUN_TO_POSITION
            drive.set_left_mode(DcMotor.RunMode.RUN_USING_ENCODER);
            drive.set_right_mode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);   // optional pause after each move.
        }
    }
}
