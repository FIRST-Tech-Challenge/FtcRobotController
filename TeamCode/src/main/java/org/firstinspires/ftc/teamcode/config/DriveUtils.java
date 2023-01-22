package org.firstinspires.ftc.teamcode.config;

import static android.os.SystemClock.sleep;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import static org.firstinspires.ftc.teamcode.config.BaseOpMode.*;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


public class DriveUtils {

    private static String position = "";
    /**
     * Logs data.
     *
     * @param caption The caption.
     * @param text    The text that needs to be logged.
     */
    public static void logData(String caption, String text) {
        telemetry.addData(caption, text);
        telemetry.update();
    }

    public static void initiate(BaseOpMode baseOpMode)
    {
        while (!baseOpMode.isStopRequested() && !baseOpMode.getRobot().imu.isGyroCalibrated())
        {
            sleep(50);
            baseOpMode.idle();
        }
    }

    public static void setPosition(String positionChange) {
        position = positionChange;
    }

    public static String getPosition()
    {
        return position;
    }
    static Orientation lastAngles = new Orientation();
    static double globalAngle;
    double power = .50;
    double correction;
    /**
     * Logs a line.
     *
     * @param line The line to be logged.
     */
    public static void logLine(String line) {
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
    public static void moveClaw(BaseOpMode BaseOpMode, String direction){
        Hardware2 robot = BaseOpMode.getRobot();
        if (direction.equalsIgnoreCase("Open")) {
            robot.getLeftClaw().setPosition(0.7);
            robot.getRightClaw().setPosition(0.3);
        } else if (direction.equalsIgnoreCase("Close")) {
            robot.getLeftClaw().setPosition(0);
            robot.getRightClaw().setPosition(1);
        }
    }

    public static void encoderDrive(BaseOpMode BaseOpMode, double speed,
                                    double leftInches, double rightInches,
                                    double timeoutS) {
//        logData( "Encoder Drive data", String.format("Speed=.2f, leftInches=.2f, rightInches=.2f, timeout=.2f",
//                speed, leftInches, rightInches, timeoutS));
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
        if (BaseOpMode.opModeIsActive()) {
            Hardware2 robot = BaseOpMode.getRobot();
            // Determine new target position, and pass to motor controller
            backLeftTarget = robot.getBackLeftDrive().getCurrentPosition() + (int) (rightInches / FTCConstants.COUNTS_PER_INCH);
            backRightTarget = robot.getBackRightDrive().getCurrentPosition() + (int) (leftInches / FTCConstants.COUNTS_PER_INCH);
            rightTarget = robot.getRightDrive().getCurrentPosition() + (int) (rightInches / FTCConstants.COUNTS_PER_INCH);
            leftTarget = robot.getLeftDrive().getCurrentPosition() + (int) (leftInches / FTCConstants.COUNTS_PER_INCH);
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
            //telemetry
            while (BaseOpMode.opModeIsActive() && (runtime.seconds() < timeoutS) &&
                    (robot.getLeftDrive().isBusy() && robot.getRightDrive().isBusy())) {
                // Display it for the driver.
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
            sleep(100);   // optional pause after each move
        }

    }

    public static void encoderStrafe(BaseOpMode BaseOpMode, double speed,
                                    double inches,
                                    double timeoutS) {
//        logData( "Encoder Drive data", String.format("Speed=.2f, leftInches=.2f, rightInches=.2f, timeout=.2f",
//                speed, leftInches, rightInches, timeoutS));
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
        if (BaseOpMode.opModeIsActive()) {
            Hardware2 robot = BaseOpMode.getRobot();
            // Determine new target position, and pass to motor controller
            backLeftTarget = robot.getBackLeftDrive().getCurrentPosition() + (int) (-inches / FTCConstants.COUNTS_PER_INCH);
            backRightTarget = robot.getBackRightDrive().getCurrentPosition() + (int) (inches / FTCConstants.COUNTS_PER_INCH);
            rightTarget = robot.getRightDrive().getCurrentPosition() + (int) (-inches / FTCConstants.COUNTS_PER_INCH);
            leftTarget = robot.getLeftDrive().getCurrentPosition() + (int) (inches / FTCConstants.COUNTS_PER_INCH);
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
            //telemetry
            while (BaseOpMode.opModeIsActive() && (runtime.seconds() < timeoutS) &&
                    (robot.getLeftDrive().isBusy() && robot.getRightDrive().isBusy())) {
                // Display it for the driver.
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
            sleep(100);   // optional pause after each move
        }

    }
    public static void encoderClaw(BaseOpMode baseOpMode, double speed, int encoderTicks, int timeoutS) {

        int target;

        final ElapsedTime runtime = new ElapsedTime();

        if(baseOpMode.opModeIsActive()) {
            Hardware2 robot = baseOpMode.getRobot();

            robot.getArm().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            target = robot.getArm().getCurrentPosition() - (encoderTicks);

            robot.getArm().setTargetPosition(target);

            robot.getArm().setMode(DcMotor.RunMode.RUN_TO_POSITION);

            runtime.reset();

            robot.getArm().setPower(speed);

            while (baseOpMode.opModeIsActive() && (runtime.seconds() < timeoutS) && (robot.getArm().isBusy())) {

            }
            robot.getArm().setPower(0);

            robot.getArm().setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);
        }
    }
    private static void resetAngle(BaseOpMode baseOpMode)
    {
        lastAngles = baseOpMode.getRobot().imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }
    public static double getAngle(BaseOpMode baseOpMode)
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = baseOpMode.getRobot().imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }
    private double checkDirection(BaseOpMode baseOpMode)
    {
        // The gain value determines how sensitive the correction is to direction changes.
        // You will have to experiment with your robot to get small smooth direction changes
        // to stay on a straight line.
        double correction, angle, gain = .01;  // was .10

        angle = getAngle(baseOpMode);

        if (angle == 0)
            correction = 0;             // no adjustment.
        else
            correction = -angle;        // reverse sign of angle for correction.

        correction = correction * gain;

        return correction;
    }
    public static void rotate(int degrees, double power, BaseOpMode baseOpMode)
    {
        double  leftPower, rightPower;

        // restart imu movement tracking.
        resetAngle(baseOpMode);

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        if (degrees < 0)
        {   // turn right.
            leftPower = power;
            rightPower = -power;
        }
        else if (degrees > 0)
        {   // turn left.
            leftPower = -power;
            rightPower = power;
        }
        else return;

        // set power to rotate.
        baseOpMode.getRobot().backLeftMotor.setPower(leftPower);
        baseOpMode.getRobot().backRightMotor.setPower(rightPower);
        baseOpMode.getRobot().frontLeftMotor.setPower(leftPower);
        baseOpMode.getRobot().frontRightMotor.setPower(rightPower);

        // rotate until turn is completed.
        if (degrees < 0)
        {
            // On right turn we have to get off zero first.
            while (baseOpMode.opModeIsActive() && getAngle(baseOpMode) == 0) {}

            while (baseOpMode.opModeIsActive() && getAngle(baseOpMode) > degrees) {}
        }
        else    // left turn.
            while (baseOpMode.opModeIsActive() && getAngle(baseOpMode) < degrees) {}

        // turn the motors off.
        baseOpMode.getRobot().backLeftMotor.setPower(0);
        baseOpMode.getRobot().backRightMotor.setPower(0);
        baseOpMode.getRobot().frontLeftMotor.setPower(0);
        baseOpMode.getRobot().frontRightMotor.setPower(0);


         sleep(250);
        // reset angle tracking on new heading.
        resetAngle(baseOpMode);
    }




}


