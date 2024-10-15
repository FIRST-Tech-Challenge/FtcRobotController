package org.firstinspires.ftc.teamcode.CompBot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Mekanism.*;
import org.firstinspires.ftc.teamcode.Swerve.*;

@TeleOp(name = "CompBot Swerve", group = "CompBot")
public class CompBot extends LinearOpMode {


    // Main swerve stuff is here
    Swerve_components swerve;

    // Main mechanism stuff is here
    Mekanism mek;


    // In case builders are bad, is offset center for servo
    double FLServoOffSet = .005;
    double FRServoOffSet = .00;
    double BLServoOffSet = .01;
    double BRServoOffSet = 0.007;


    // Robot dimensions
    static double TRACK_WIDTH = 14; // In inches
    static double WHEELBASE = 15; // In inches


    /**
     * Controls for game pad 1:<br>
     * Right trigger: forwards<br>
     * Left trigger: backwards<br>
     * Right stick x: rotate<br>
     * Left stick x: strafe<br>
     * <p>
     * Controls for game pad 2:<br>
     * Left stick y: in and out of arm<br>
     * Right stick y: up and down of arm<br>
     * Left trigger: claw intake<br>
     * Right trigger: claw out<br>
     * <p>
     * Presets for:<br>
     * Attaching clip to sample<br>
     * Attaching specimen(clip + sample) to top rung<br>
     * Presets for bucket 1 and 2
     */
    public void runOpMode() throws InterruptedException {

        swerve.initSwerve(); // Inits the swerve components
        mek.initMekanism(); // Inits the mechanism components


        waitForStart();
        while (opModeIsActive()) {

            //game pad 1
            double speedGMP1 = gamepad1.left_trigger - gamepad1.right_trigger; // Makes it so that the triggers cancel each other out if both are pulled at the same time
            double angleGMP1 = -gamepad1.right_stick_x;

            if (speedGMP1 != 0) move(gamepad1.left_stick_x, speedGMP1);
            else if (angleGMP1 != 0) rotate(angleGMP1);
            else {
                swerve.FLMotor.setPower(0);
                swerve.BLMotor.setPower(0);
                swerve.BRMotor.setPower(0);
                swerve.FRMotor.setPower(0);
            }
            // if (gamepad1.a) rotateToCenter();


            // Game pad 2
            double armLength = -gamepad2.right_stick_y;
            double armAngle = -gamepad2.left_stick_y;

            // 0 < arm length < 4268
            if (armLength > 0 && mek.slide.getCurrentPosition() > 4268) {
                armLength = 0;
            } else if (armLength < 0 && mek.slide.getCurrentPosition() < -4268) {
                armLength = 0;
            }
            mek.slide.setPower(armLength);

            // 0 < arm angle < 2904
            if (armAngle > 0 && mek.pivot.getCurrentPosition() > 2904) {
                armAngle = 0;
            } else if (armAngle < 0 && mek.pivot.getCurrentPosition() < -2904) {
                armAngle = 0;
            }
            mek.pivot.setPower(armAngle);

            // To test arm length and angle
            mekTelemetry(mek.pivot.getCurrentPosition(), mek.slide.getCurrentPosition(), armAngle, armLength);
        }
    }


    /**
     * Prints out telemetry for the mechanism
     */
    public void mekTelemetry(int x, int y, double a, double b) {
        telemetry.addData("Arm angle: ", x);
        telemetry.addData("Arm length: ", y);
        telemetry.addData("Left stick y: ", a);
        telemetry.addData("Right stick y: ", b);
        telemetry.update();
    }


    /**
     * Converts standard cartesian coordinates to polar coordinates
     *
     * @param x X input
     * @param y Y input
     * @return Returns an array of two doubles,<br>
     * <p>
     * [0] = R - Magnitude<br>
     * [1] = Theta Angle of input coordinate.<br>
     * Relative to unit circle, where 0deg in is to the right, 90 is up and 180 is left.
     * @see #polarToCartesian(double, double)
     */
    public double[] cartesianToPolar(double x, double y) {
        double[] arrayToReturn = new double[2];
        arrayToReturn[0] = Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2)); // Radius
        arrayToReturn[1] = Math.atan(y / x) * (Math.PI / 180); // Theta

        return arrayToReturn;
    }


    /**
     * Converts polar coordinates to cartesian
     *
     * @param r     Magnitude of input coordinate
     * @param theta Angle of input coordinate.<br>
     *              Relative to unit circle, where 0deg in is to the right, 90 is up and 180 is left.
     * @return Returns an array of two doubles,<br>
     * <p>
     * [0] = X<br>
     * [1] = Y
     * @see #cartesianToPolar(double, double)
     */
    public double[] polarToCartesian(double r, double theta) {
        double[] arrayToReturn = new double[2];
        arrayToReturn[0] = r * Math.cos(theta); // X
        arrayToReturn[1] = r * Math.sin(theta); // Y

        return arrayToReturn;
    }


    /**
     * TODO Move to Swerve_Components as it only deals with swerve drive stuff
     * <p>
     * Moves the robot based on desired heading and power.<br>
     * Does not change the rotation of the robot at all
     *
     * @param heading Desired heading of the robot.<br>
     *                0 is straight forward 1 is fully right, -1 is fully left
     * @param power   Desired power to run the motors at
     */
    public void move(double heading, double power) {
        heading = (heading + 1) / 2;

        swerve.FLServo.setPosition(heading + FLServoOffSet);
        swerve.BLServo.setPosition(heading + BLServoOffSet);
        swerve.BRServo.setPosition(heading + BRServoOffSet);
        swerve.FRServo.setPosition(heading + FRServoOffSet);

        swerve.FLMotor.setPower(power);
        swerve.BLMotor.setPower(power);
        swerve.BRMotor.setPower(power);
        swerve.FRMotor.setPower(power);
    }


    /**
     * TODO This needs to be moved to Swerve_Components
     * <p>
     * Rotates the robot around a center point.<br>
     * Does not move the robot in any other direction.<br>
     *
     * @param power Power to turn the robot at
     */
    public void rotate(double power) {

        // Set wheels for rotation (Ben's robot has 2x gear ratio so .25/2 and .75/2)
        swerve.FLServo.setPosition(.25 + .125 / 2);
        swerve.BLServo.setPosition(.75 - .125 / 2);
        swerve.BRServo.setPosition(.25 + .125 / 2);
        swerve.FRServo.setPosition(.75 - .125 / 2);

        //turn motors to rotate robot
        swerve.FLMotor.setPower(-power);
        swerve.BLMotor.setPower(-power);
        swerve.BRMotor.setPower(power);
        swerve.FRMotor.setPower(power);

    }


    /**
     * TODO This needs to be worked on. Nothing is done here
     *
     * <p>
     * Wheel angle is perpendicular to turn angle
     * turn angle is inverse tan(get angle) of 7.5(half of wheel base length) / (turning distance/2)
     * because it is radius of point we are trying to rotate around
     * speed = -1 to 1
     * turnRad = -1 to 1
     * turnDir = LEFT or RIGHT
     */
    public void moveAndRotate(double speed, double turnAmount) {

        double i_WheelAngle = Math.atan2(WHEELBASE, turnAmount - TRACK_WIDTH / 2);
        double outsideAng = Math.atan2(WHEELBASE, turnAmount + TRACK_WIDTH / 2);


    }


    /**
     * TODO Possibly make this not run at full speed at all times by adding some sort of power input
     * <p>
     * Uses the IMU to move the robot to face forward
     * <p>
     * As long as this function is called, it will try to rotate back to facing forward
     */
    public void rotateToCenter() {
        double orientation = swerve.odo.getHeading();
        telemetry.addData("Yaw angle", orientation);

        rotate(orientation);
    }


    /**
     * TODO All of this
     * Moves the robot to the detected specimen
     */
    public void moveToSpecimen() {

    }


    /**
     * TODO All of this as well
     * Moves the robot back to the storage area
     */
    public void moveToStore() {

    }

}
