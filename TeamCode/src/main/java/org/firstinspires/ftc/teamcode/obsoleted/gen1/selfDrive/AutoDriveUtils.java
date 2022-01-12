package org.firstinspires.ftc.teamcode.obsoleted.gen1.selfDrive;

import static org.firstinspires.ftc.teamcode.common.utils.FTCConstants.COUNTS_PER_INCH;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.obsoleted.gen1.BaseOpMode;
import org.firstinspires.ftc.teamcode.obsoleted.gen1.Hardware2;

/**
 * This class implements necessary utility methods for autonomous driving.
 * @author aryansinha
 */
public final class AutoDriveUtils {
    /**
     * Logs data.
     * @param baseOpMode The base op mode.
     * @param caption The caption.
     * @param text The text that needs to be logged.
     */
    public static void logData(BaseOpMode baseOpMode, String caption, String text) {
        baseOpMode.telemetry.addData(caption, text);
        baseOpMode.telemetry.update();
    }

    /**
     * Logs a line.
     * @param baseOpMode The base op mode.
     * @param line The line to be logged.
     */
    public static void logLine(BaseOpMode baseOpMode, String line) {
        baseOpMode.telemetry.addLine(line);
        baseOpMode.telemetry.update();
    }


    /*
     *  Method to perfmorm a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    public static void encoderDrive(BaseOpMode baseOpMode, double speed,
                                    double leftInches, double rightInches,
                                    double timeoutS) {
        logData(baseOpMode, "Encoder Drive data", String.format("Speed=.2f, leftInches=.2f, rightInches=.2f, timeout=.2f",
                speed, leftInches, rightInches, timeoutS));
        // DEFINE 4 variables of type int
        // int nameOfVariable;
        // The variable names should be:
        // newBackLeftTarget, newBackRightTarget, newFrontLeftTarget, newFrontRightTarget
        int leftTarget;
        int rightTarget;
        final ElapsedTime runtime = new ElapsedTime();

        // Ensure that the opmode is still active
        if (baseOpMode.opModeIsActive()) {
            Hardware2 robot = baseOpMode.getRobot();
            // Determine new target position, and pass to motor controller
            leftTarget = robot.getLeftDrive().getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            rightTarget = robot.getRightDrive().getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            // set the targetPosition for each motor
            // call the motor's setPosition() method and pass it new target value
            robot.getLeftDrive().setTargetPosition(leftTarget);
            robot.getRightDrive().setTargetPosition(rightTarget);


            // Turn On RUN_TO_POSITION
            robot.getLeftDrive().setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.getRightDrive().setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.getLeftDrive().setPower(Math.abs(speed));
            robot.getRightDrive().setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.

            while ( baseOpMode.opModeIsActive() && (runtime.seconds() < timeoutS) &&
                    (robot.getLeftDrive().isBusy() && robot.getRightDrive().isBusy())) {

                // Display it for the driver.
                logData(baseOpMode,"Path1", String.format("Running to %7d :%7d", leftTarget,  rightTarget));
                logData(baseOpMode,"Path2", String.format("Running at %7d :%7d", robot.getLeftDrive().getCurrentPosition(), robot.getRightDrive().getCurrentPosition()));
            }

            // Stop all motion (set the power of each motor to 0)
            robot.getLeftDrive().setPower(0);
            robot.getRightDrive().setPower(0);

            // Reset all motors and Turn off RUN_TO_POSITION
            robot.getLeftDrive().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.getRightDrive().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            //  sleep(250);   // optional pause after each move
        }
    }
}
