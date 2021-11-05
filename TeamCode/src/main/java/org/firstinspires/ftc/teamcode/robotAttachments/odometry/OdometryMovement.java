package org.firstinspires.ftc.teamcode.robotAttachments.odometry;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;


public class OdometryMovement {
    DcMotor front_right, front_left, back_right, back_left;
    Telemetry telemetry;
    OdometryGlobalCoordinatePosition odometry;
    Executable<Boolean> _isStopRequested;
    Executable<Boolean> _opModeIsActive;

    public OdometryMovement(DcMotor front_right, DcMotor front_left, DcMotor back_right, DcMotor back_left, Telemetry telemetry, OdometryGlobalCoordinatePosition odometry, Executable<Boolean> isStopRequested, Executable<Boolean> opmodeIsActive) {
        this.back_left = back_left;
        this.back_right = back_right;
        this.front_left = front_left;
        this.front_right = front_right;
        this.telemetry = telemetry;
        this.odometry = odometry;
        this._isStopRequested = isStopRequested;
        this._opModeIsActive = opmodeIsActive;
    }

    boolean isStopRequested() throws InterruptedException {
        if (_isStopRequested.call()) {
            throw new InterruptedException();
        }
        return false;
    }

    boolean opModeIsActive() {
        return _opModeIsActive.call();
    }


    /**
     * @param x         X Value to move to
     * @param y         Y Value to move to
     * @param tolerance The distance the robot can be off from the given position
     */
    public void moveToPosition(double x, double y, double tolerance) throws InterruptedException {
        moveToPosition(x, y, tolerance, false);
    }

    /**
     * This is used to get the angle between two points
     *
     * @param rx       The robot x position
     * @param ry       Robot Y Position
     * @param x        X Position to go to
     * @param y        Y position to go to
     * @param robotRot The orientation of the robot
     * @return The heading the point is from the robot
     */
    public static double getAngle(double rx, double ry, double x, double y, double robotRot) {
        double angle;
        x = x - rx;
        y = y - ry;
        angle = Math.toDegrees(Math.atan2(x, y));
        return ((angle - robotRot) % 360);
    }


    /**
     * @param x         The x position to move to
     * @param y         The y position to move to
     * @param tolerance The tolerence for the movement
     */
    private void preciseMovement(double x, double y, double tolerance) throws InterruptedException {
        double power = 0.1;
        final String s = x + " , " + y;
        while (distance(odometry.returnXCoordinate() / 1892.3724283364, odometry.returnYCoordinate() / 1892.3724283364, x, y) > tolerance && !isStopRequested()) {
            telemetry.addData("Moving to", s);
            telemetry.update();

            strafeAtAngle(getAngle(odometry.returnXCoordinate() / 1892.3724283364, odometry.returnYCoordinate() / 1892.3724283364, x, y, odometry.returnOrientation()), 0.5);

        }
        front_left.setPower(0);
        front_right.setPower(0);
        back_right.setPower(0);
        back_left.setPower(0);
    }


    /**
     * @param x1 the x-value of the first point
     * @param y1 the y-value of the first point
     * @param x2 the x-value of the second point
     * @param y2 the y-value of the second point
     * @return The distance between two points
     */
    public static double distance(double x1, double y1, double x2, double y2) {
        return Math.sqrt((y2 - y1) * (y2 - y1) + (x2 - x1) * (x2 - x1));
    }


    /**
     * This assumes the pattern of drive wheels unique to the Ultimate Goal robot
     * The set Motor power section may need to be changed
     *
     * @param x             X Value to move to
     * @param y             Y Value to move to
     * @param tolerance     The distance the robot can be off from the given position
     * @param consoleOutput Prints debug info to the console for debugging, is slower and less accurate
     */
    public void moveToPosition(double x, double y, double tolerance, boolean consoleOutput) throws InterruptedException {
        final String s = x + " , " + y;
        double power = 0;
        //by setting distance to max value, we make sure that the loop will execute once
        //by recalculating distance in the loop rather than in the while parenthesises, we remove one distance() call
        double distance = Double.MAX_VALUE;
        double odometry_angle;
        double odometry_x;
        double odometry_y;

        while (distance > tolerance && !isStopRequested() && opModeIsActive()) {
            //By calculating the values here once in this loop and declaring the variables above, we minimize the number
            //of memory allocation calls and the number of variable calculations.
            odometry_x = odometry.returnXCoordinate() / 1892.3724283364; //odometry x
            odometry_y = odometry.returnYCoordinate() / 1892.3724283364; //odometry y
            distance = distance(odometry_x, odometry_y, x, y); //distance value
            odometry_angle = getAngle(odometry_x, odometry_y, x, y, odometry.returnOrientation()); //angle

            /*The next if-else block takes the distance from target and
             sets the power variable to odometry_angle power following the function
             @param zeroPoint is the point where the robot goes at 1
             power = 0.8/zeroPoint(distance) + 0.2
             if the distance is greater than 24 in or ~2 ft, robot moves at power of 1
             */
            final double zeroPoint = 24;
            if (distance > zeroPoint) {
                power = 1;
            } else {
                power = ((0.9 / zeroPoint) * distance) + 0.2;
            }
            //power = Math.abs(power);


            if (consoleOutput) {

                telemetry.addData("Moving to", s);
                telemetry.addData("distance", distance);
                telemetry.addData("angle", odometry_angle);
                telemetry.addData("Moving?", distance > tolerance);
                telemetry.addData("X Pos", odometry_x);
                telemetry.addData("Y Pos", odometry_y);
                telemetry.addData("Power", power);
                telemetry.update();

            }
            strafeAtAngle(odometry_angle, power);

        }
        front_left.setPower(0);
        front_right.setPower(0);
        back_right.setPower(0);
        back_left.setPower(0);
    }

    /**
     * This makes the robot strafe at an heading at a power
     * Assumes the drive motor configuration of the Ultimate Goal robot
     * The set power section may need to be updated
     *
     * @param angle The angle to strafe at, 0 degrees is straight, 90 degrees is to the right
     * @param power The power to strafe at
     */
    public void strafeAtAngle(double angle, double power) {
        power = boundNumber(power);
        double power1 = 0;
        double power2 = 0;
        angle = angle % 360;

        power1 = Math.cos(Math.toRadians(angle + 45.0));
        power2 = Math.cos(Math.toRadians(angle - 45));

        power1 = power * power1;
        power2 = power * power2;

        front_right.setPower(power1);
        back_left.setPower(power1);

        front_left.setPower(power2);
        back_right.setPower(-power2);

    }

    private static double boundNumber(double num) {
        if (num > 1) {
            num = 1;
        }
        if (num < -1) {
            num = -1;
        }
        return num;
    }


}
