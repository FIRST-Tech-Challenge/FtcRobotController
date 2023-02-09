package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class DrivebaseMethods {
    Hardware robot;

    ElapsedTime runtime;

    public DrivebaseMethods(Hardware r) {
        robot = r;
        runtime = new ElapsedTime();
    }

    public void move(Gamepad gamepad) {
        robot.frontLeftMotor.setPower((gamepad.left_stick_y) + gamepad.right_stick_x - gamepad.left_stick_x);
        robot.backLeftMotor.setPower((gamepad.left_stick_y) + (gamepad.right_stick_x) + gamepad.left_stick_x);
        robot.backRightMotor.setPower((gamepad.left_stick_y) - gamepad.right_stick_x - gamepad.left_stick_x);
        robot.frontRightMotor.setPower((gamepad.left_stick_y) - (gamepad.right_stick_x) + gamepad.left_stick_x);
    }
    public void move(Gamepad gamepad, double dampeningFactor) {
        robot.frontLeftMotor.setPower(dampeningFactor * ((gamepad.left_stick_y) + gamepad.right_stick_x - gamepad.left_stick_x));
        robot.backLeftMotor.setPower(dampeningFactor * ((gamepad.left_stick_y) + (gamepad.right_stick_x) + gamepad.left_stick_x));
        robot.backRightMotor.setPower(dampeningFactor * ((gamepad.left_stick_y) - gamepad.right_stick_x - gamepad.left_stick_x));
        robot.frontRightMotor.setPower(dampeningFactor * ((gamepad.left_stick_y) - (gamepad.right_stick_x) + gamepad.left_stick_x));
    }

    public void fieldCentric(Gamepad gamepad) {
        double y = -gamepad.left_stick_y; // Remember, this is reversed!
        double x = gamepad.left_stick_x * 1.1; // Counteract imperfect strafing
        double rx = gamepad.right_stick_x;

        // Read inverse IMU heading, as the IMU heading is CW positive
        double botHeading = -getAngleRadians();

        double rotX = x * Math.cos(botHeading) - y * Math.sin(botHeading);
        double rotY = x * Math.sin(botHeading) + y * Math.cos(botHeading);

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio, but only when
        // at least one is out of the range [-1, 1]

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX - rx) / denominator;
        double backLeftPower = (rotY - rotX - rx) / denominator;
        double frontRightPower = (rotY - rotX + rx) / denominator;
        double backRightPower = (rotY + rotX + rx) / denominator;

        robot.frontLeftMotor.setPower(-1 * frontLeftPower);
        robot.backLeftMotor.setPower(-1 * backLeftPower);
        robot.backRightMotor.setPower(-1 * backRightPower);
        robot.frontRightMotor.setPower(-1 * frontRightPower);

    }

    public void setDriveMotorsToTrue() {
        robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void setDriveMotorsToFalse() {
        robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void encoderDrive(String direction, double centimeters, double power) {
        //ticks is equal to (distance / circumference) * (encoder counts per wheel revolution)
        if (!robot.encoder) {
            throw new IllegalArgumentException("Encoder not set to true");
        }
        int ticks = (int) ((centimeters / (Math.PI * 9.6)) * 537.7);


        robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Switched from original Hardware class
        robot.frontLeftMotor.setDirection(DcMotor.Direction.FORWARD); // Originally Reverse
        robot.frontRightMotor.setDirection(DcMotor.Direction.REVERSE);//Originally Forward
        robot.backLeftMotor.setDirection(DcMotor.Direction.FORWARD); //Originally Reverse
        robot.backRightMotor.setDirection(DcMotor.Direction.REVERSE);//Originally Forward

        if (direction.equals("Front")) {
            int rightFrontTargetPos = robot.frontRightMotor.getCurrentPosition() + ticks;
            int leftFrontTargetPos = robot.frontLeftMotor.getCurrentPosition() + ticks;
            int rightBackTargetPos = robot.backRightMotor.getCurrentPosition() + ticks;
            int leftBackTargetPos = robot.backLeftMotor.getCurrentPosition() + ticks;

            robot.frontRightMotor.setTargetPosition(rightFrontTargetPos);
            robot.frontLeftMotor.setTargetPosition(leftFrontTargetPos);
            robot.backRightMotor.setTargetPosition(rightBackTargetPos);
            robot.backLeftMotor.setTargetPosition(leftBackTargetPos);

            setPowerOfAllMotorsTo(power);

            robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        } else if (direction.equals("Back")) {
            int rightFrontTargetPos = robot.frontRightMotor.getCurrentPosition() - ticks;
            int leftFrontTargetPos = robot.frontLeftMotor.getCurrentPosition() - ticks;
            int rightBackTargetPos = robot.backRightMotor.getCurrentPosition() - ticks;
            int leftBackTargetPos = robot.backLeftMotor.getCurrentPosition() - ticks;

            robot.frontRightMotor.setTargetPosition(rightFrontTargetPos);
            robot.frontLeftMotor.setTargetPosition(leftFrontTargetPos);
            robot.backRightMotor.setTargetPosition(rightBackTargetPos);
            robot.backLeftMotor.setTargetPosition(leftBackTargetPos);

            setPowerOfAllMotorsTo(power);

            robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        } else if (direction.equals("Right")) {
            int rightFrontTargetPos = robot.frontRightMotor.getCurrentPosition() - 2 * ticks;
            int leftFrontTargetPos = robot.frontLeftMotor.getCurrentPosition() + 2 * ticks;
            int rightBackTargetPos = robot.backRightMotor.getCurrentPosition() + 2 * ticks;
            int leftBackTargetPos = robot.backLeftMotor.getCurrentPosition() - 2 * ticks;

            robot.frontRightMotor.setTargetPosition(rightFrontTargetPos);
            robot.frontLeftMotor.setTargetPosition(leftFrontTargetPos);
            robot.backRightMotor.setTargetPosition(rightBackTargetPos);
            robot.backLeftMotor.setTargetPosition(leftBackTargetPos);

            setPowerOfAllMotorsTo(power);

            robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        } else if (direction.equals("Left")){
            int rightFrontTargetPos = robot.frontRightMotor.getCurrentPosition() + 2 * ticks;
            int leftFrontTargetPos = robot.frontLeftMotor.getCurrentPosition() - 2 * ticks;
            int rightBackTargetPos = robot.backRightMotor.getCurrentPosition() - 2 * ticks;
            int leftBackTargetPos = robot.backLeftMotor.getCurrentPosition() + 2 * ticks;

            robot.frontRightMotor.setTargetPosition(rightFrontTargetPos);
            robot.frontLeftMotor.setTargetPosition(leftFrontTargetPos);
            robot.backRightMotor.setTargetPosition(rightBackTargetPos);
            robot.backLeftMotor.setTargetPosition(leftBackTargetPos);

            setPowerOfAllMotorsTo(power);

            robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        } else {
            setPowerOfAllMotorsTo(0);
        }
    }

    public void setPowerOfAllMotorsTo(double power) {
        robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        robot.frontRightMotor.setPower(power);
        robot.frontLeftMotor.setPower(power);
        robot.backRightMotor.setPower(power);
        robot.backLeftMotor.setPower(power);
    }

    public void setPowerOfIndividualMotorsTo(double frontRightPower, double frontLeftPower, double backRightPower, double backLeftPower) {
        robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        robot.frontRightMotor.setPower(frontRightPower);
        robot.frontLeftMotor.setPower(frontLeftPower);
        robot.backRightMotor.setPower(backRightPower);
        robot.backLeftMotor.setPower(backLeftPower);
    }

    /**
     * Resets the cumulative angle tracking to zero.
     */
    public void resetAngle()
    {
        robot.lastAngles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        robot.globalAngle = 0;
    }

    /**
     * Get current cumulative angle rotation from last reset.
     * @return Angle in degrees. + = left, - = right.
     */
    public double getAngle()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - robot.lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        robot.globalAngle += deltaAngle;

        robot.lastAngles = angles;

        return robot.globalAngle;
    }

    public double getAngleRadians() {

        return robot.imu.getAngularOrientation().firstAngle;
    }

    /**
     * Rotate left or right the number of degrees. Does not support turning more than 180 degrees.
     * @param degrees Degrees to turn, + is left - is right
     */
    public void rotate(int degrees, double power)
    {
        double  leftPower, rightPower;

        // restart imu movement tracking.
        resetAngle();


        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        if (degrees < 0)
        {   // turn right.
            leftPower = -power;
            rightPower = power;
        }
        else if (degrees > 0)
        {   // turn left.
            leftPower = power;
            rightPower = -power;
        }
        else return;

        // set power to rotate.
        robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        robot.frontRightMotor.setPower(rightPower);
        robot.frontLeftMotor.setPower(leftPower);
        robot.backRightMotor.setPower(rightPower);
        robot.backLeftMotor.setPower(leftPower);

        // rotate until turn is completed.
        if (degrees < 0)
        {
            // On right turn we have to get off zero first.
            while (getAngle() == 0) {
//                /*telemetry.addData("Direction", checkDirection());
                //telemetry.addData("Angle", getAngle());
                //telemetry.update();
            }

            while (getAngle() > degrees) {
                /*telemetry.addData("Direction", checkDirection());*/
                //telemetry.addData("Angle", getAngle());
                //telemetry.update();
            }
        }
        else    // left turn.
            while (getAngle() < degrees) {
                /*telemetry.addData("Direction", checkDirection());*/
                //telemetry.addData("Angle", getAngle());
                //telemetry.update();
            }

        // turn the motors off.
        robot.frontRightMotor.setPower(0);
        robot.frontLeftMotor.setPower(0);
        robot.backRightMotor.setPower(0);
        robot.backLeftMotor.setPower(0);

        // reset angle tracking on new heading.
        resetAngle();
    }

    public void rotateTest(int degrees, double power)
    {
        double  leftPower, rightPower;

        // restart imu movement tracking.
        resetAngle();


        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        if (degrees < 0)
        {   // turn right.
            leftPower = -power;
            rightPower = power;
        }
        else if (degrees > 0)
        {   // turn left.
            leftPower = power;
            rightPower = -power;
        }
        else return;

        // set power to rotate.
        robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        robot.frontRightMotor.setPower(rightPower);
        robot.frontLeftMotor.setPower(leftPower);
        robot.backRightMotor.setPower(rightPower);
        robot.backLeftMotor.setPower(leftPower);

        // rotate until turn is completed.
        if (degrees < 0)
        {
            // On right turn we have to get off zero first.
            if (getAngle() == 0) {
                leftPower = power;
                rightPower = -power;
            }

            while (getAngle() > degrees) {
                /*telemetry.addData("Direction", checkDirection());*/
                //telemetry.addData("Angle", getAngle());
                //telemetry.update();
            }
        }
        else { // left turn.
            if (getAngle() < degrees) {
                leftPower = -power;
                rightPower = power;
            }
        }

        if (getAngle() == degrees) {
            leftPower = 0;
            rightPower = 0;
        }

        // turn the motors off.
        robot.frontRightMotor.setPower(rightPower);
        robot.frontLeftMotor.setPower(leftPower);
        robot.backRightMotor.setPower(rightPower);
        robot.backLeftMotor.setPower(leftPower);

        // reset angle tracking on new heading.
        resetAngle();
    }

    public void turnWithPID(double targetAngle, double kp, double ki, double kd, double threshold) {
        // P is responsible for changing the speed relative to the error
        // I is responsible for the accumulated error (I think)
        // D makes the response to the error an exponential curve, rather than a line

        double error = targetAngle - getAngle();
        double sum = error;
        double previousError = error;
        double correction = 0;
        double slope = 0;
        runtime.reset();
        while (Math.abs(error) > threshold) {
            //telemetry.addData("error", error);
            //telemetry.update();
            error = targetAngle - getAngle();
            slope = (error - previousError) / (double) runtime.time();
            runtime.reset();
            sum += error;
            correction = kp * error + ki * sum + kd * slope;


            robot.frontRightMotor.setPower(correction);
            robot.frontLeftMotor.setPower(-1 * correction);
            robot.backRightMotor.setPower(correction);
            robot.backLeftMotor.setPower(-1 * correction);
        }
    }

    public void rotateWithoutReset(int degrees, double power)
    {
        double  leftPower, rightPower;

        // restart imu movement tracking.
        //resetAngle();
        int angle = (int)(degrees - getAngle());


        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        if (angle < 0)
        {   // turn right.
            leftPower = -power;
            rightPower = power;
        }
        else if (angle > 0)
        {   // turn left.
            leftPower = power;
            rightPower = -power;
        }
        else return;

        // set power to rotate.
        robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        robot.frontRightMotor.setPower(rightPower);
        robot.frontLeftMotor.setPower(leftPower);
        robot.backRightMotor.setPower(rightPower);
        robot.backLeftMotor.setPower(leftPower);

        // rotate until turn is completed.
        if (angle < 0)
        {
            // On right turn we have to get off zero first.
            while (getAngle() == 0) {
//                /*telemetry.addData("Direction", checkDirection());
                //telemetry.addData("Angle", getAngle());
                //telemetry.update();
            }

            while (getAngle() > degrees) {
                /*telemetry.addData("Direction", checkDirection());*/
                //telemetry.addData("Angle", getAngle());
                //telemetry.update();
            }
        }
        else    // left turn.
            while (getAngle() < degrees) {
                /*telemetry.addData("Direction", checkDirection());*/
                //telemetry.addData("Angle", getAngle());
                //telemetry.update();
            }

        // turn the motors off.
        robot.frontRightMotor.setPower(0);
        robot.frontLeftMotor.setPower(0);
        robot.backRightMotor.setPower(0);
        robot.backLeftMotor.setPower(0);

        // reset angle tracking on new heading.
        //resetAngle();
    }


    //public void turnWithWheels
}
