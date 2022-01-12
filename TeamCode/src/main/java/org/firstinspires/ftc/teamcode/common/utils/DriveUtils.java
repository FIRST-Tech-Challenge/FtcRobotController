package org.firstinspires.ftc.teamcode.common.utils;

import static org.firstinspires.ftc.teamcode.common.utils.FTCConstants.COUNTS_PER_INCH;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.BaseNewOpMode;
import org.firstinspires.ftc.teamcode.config.HardwareNew;

/**
 * This class implements necessary utility methods for autonomous driving.
 * @author aryansinha
 * And less significantly,
 * Very less significantly,
 * `IN SUCH AN INSIGNIFICANT WAY THAT ONLY 1 AIR MOLOCULE WAS DISPLACED BY THEM`
 * @soon-to-be-author karthikperi
 */
public final class DriveUtils {
    /**
     * Logs data.
     * @param baseNewOpMode The base op mode.
     * @param caption The caption.
     * @param text The text that needs to be logged.
     */
    public static void logData(BaseNewOpMode baseNewOpMode, String caption, String text) {
        baseNewOpMode.telemetry.addData(caption, text);
        baseNewOpMode.telemetry.update();
    }

    /**
     * Logs a line.
     * @param baseNewOpMode The base op mode.
     * @param line The line to be logged.
     */
    public static void logLine(BaseNewOpMode baseNewOpMode, String line) {
        baseNewOpMode.telemetry.addLine(line);
        baseNewOpMode.telemetry.update();
    }


    /*
     *  Method to perfmorm a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    public static void encoderDrive(BaseNewOpMode baseNewOpMode, double speed,
                                    double leftInches, double rightInches,
                                    double timeoutS) {
        logData(baseNewOpMode, "Encoder Drive data", String.format("Speed=.2f, leftInches=.2f, rightInches=.2f, timeout=.2f",
                speed, leftInches, rightInches, timeoutS));
        // DEFINE 4 variables of type int
        // int nameOfVariable;
        // The variable names should be:
        // newBackLeftTarget, newBackRightTarget, newFrontLeftTarget, newFrontRightTarget
        int backLeftTarget;
        int backRightTarget;
        int rightTarget;
        int leftTarget;
        final ElapsedTime runtime = new ElapsedTime();

        // Ensure that the opmode is still active
        if (baseNewOpMode.opModeIsActive()) {
            HardwareNew robot = baseNewOpMode.getRobot();
            // Determine new target position, and pass to motor controller
            backLeftTarget = robot.getBackLeftDrive().getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            backRightTarget = robot.getBackRightDrive().getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            rightTarget = robot.getRightDrive().getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            leftTarget = robot.getLeftDrive().getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            // set the targetPosition for each motor
            // call the motor's setPosition() method and pass it new target value
            robot.getBackLeftDrive().setTargetPosition(backLeftTarget);
            robot.getBackRightDrive().setTargetPosition(backRightTarget);
            robot.getLeftDrive().setTargetPosition(leftTarget);
            robot.getRightDrive().setTargetPosition(rightTarget);


            // Turn On RUN_TO_POSITION
            robot.getLeftDrive().setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.getRightDrive().setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.getBackLeftDrive().setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.getBackRightDrive().setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.getLeftDrive().setPower(Math.abs(speed));
            robot.getRightDrive().setPower(Math.abs(speed));
            robot.getBackLeftDrive().setPower(Math.abs(speed));
            robot.getBackRightDrive().setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.

            while ( baseNewOpMode.opModeIsActive() && (runtime.seconds() < timeoutS) &&
                    (robot.getLeftDrive().isBusy() && robot.getRightDrive().isBusy())) {

                // Display it for the driver.
                logData(baseNewOpMode,"Path1", String.format("Running to %7d :%7d", backLeftTarget,  backRightTarget));
                logData(baseNewOpMode,"Path2", String.format("Running at %7d :%7d", robot.getLeftDrive().getCurrentPosition(), robot.getRightDrive().getCurrentPosition()));
            }

            // Stop all motion (set the power of each motor to 0)
            robot.getLeftDrive().setPower(0);
            robot.getRightDrive().setPower(0);
            robot.getBackLeftDrive().setPower(0);
            robot.getBackRightDrive().setPower(0);

            // Reset all motors and Turn off RUN_TO_POSITION
            robot.getBackLeftDrive().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.getBackRightDrive().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.getLeftDrive().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.getRightDrive().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            baseNewOpMode.sleep(250);   // optional pause after each move
        }
    }
}
