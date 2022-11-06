package org.firstinspires.ftc.teamcode.config;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.config.Hardware2;

public class DriveUtils {
    /**
     * Logs data.
     * @param caption The caption.
     * @param text The text that needs to be logged.
     */
    public static void logData( String caption, String text) {
        telemetry.addData(caption, text);
        telemetry.update();
    }
    /**
     * Logs a line.
     * @param line The line to be logged.
     */
    public static void logLine( String line) {
        telemetry.addLine(line);
        telemetry.update();
    }
    /*
     *  Method to perfmorm a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */

    public static void encoderDrive( double speed,
                                     double leftInches, double rightInches,
                                     double timeoutS) {
        logData( "Encoder Drive data", String.format("Speed=.2f, leftInches=.2f, rightInches=.2f, timeout=.2f",
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

        //public static void intakeDecceleration(int ticks) {

    }
}
//)