package org.firstinspires.ftc.teamcode.src.robotAttachments.DriveTrains;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.src.Utills.Executable;
import org.firstinspires.ftc.teamcode.src.robotAttachments.odometry.OdometryGlobalCoordinatePosition;


public class OdometryDrivetrain extends BasicDrivetrain {
    Telemetry telemetry;
    OdometryGlobalCoordinatePosition odometry;
    Executable<Boolean> _isStopRequested;
    Executable<Boolean> _opModeIsActive;

    protected OdometryDrivetrain(){super();}


    public OdometryDrivetrain(DcMotor front_right, DcMotor front_left, DcMotor back_right, DcMotor back_left, Telemetry telemetry, OdometryGlobalCoordinatePosition odometry, Executable<Boolean> isStopRequested, Executable<Boolean> opmodeIsActive) {
        super(front_right,front_left,back_right,back_left);
        this.telemetry = telemetry;
        this.odometry = odometry;
        this._isStopRequested = isStopRequested;
        this._opModeIsActive = opmodeIsActive;
    }

    public OdometryDrivetrain(BasicDrivetrain drivetrain, Telemetry telemetry, OdometryGlobalCoordinatePosition odometry, Executable<Boolean> isStopRequested, Executable<Boolean> opmodeIsActive){
        super(drivetrain.front_right,drivetrain.front_left,drivetrain.back_right,drivetrain.back_left);
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
    private static double getAngle(double rx, double ry, double x, double y, double robotRot) {
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
        while (distance(odometry.returnRelativeXPosition(), odometry.returnRelativeYPosition(), x, y) > tolerance && !isStopRequested()) {
            telemetry.addData("Moving to", s);
            telemetry.update();

            strafeAtAngle(getAngle(odometry.returnRelativeXPosition(), odometry.returnRelativeXPosition(), x, y, odometry.returnOrientation()), 0.5);

        }
        stopAll();
    }


    /**
     * @param x1 the x-value of the first point
     * @param y1 the y-value of the first point
     * @param x2 the x-value of the second point
     * @param y2 the y-value of the second point
     * @return The distance between two points
     */
    private static double distance(double x1, double y1, double x2, double y2) {
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
            odometry_x = odometry.returnRelativeXPosition(); //odometry x
            odometry_y = odometry.returnRelativeYPosition(); //odometry y
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
        stopAll();
    }

    public int[] getOdometryRaw(){
        return odometry.returnRaw();
    }

    public int returnRightEncoderPosition(){
        return odometry.returnRightEncoderPosition();
    }

    public int returnLeftEncoderPosition(){
        return odometry.returnLeftEncoderPosition();
    }

    public int returnHorizontalEncoderPosition(){
        return odometry.returnHorizontalEncoderPosition();
    }
}
