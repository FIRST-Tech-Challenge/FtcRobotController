package org.firstinspires.ftc.teamcode.CompBot;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ODO.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.Mekanism.*;
import org.firstinspires.ftc.teamcode.Swerve.*;

@TeleOp(name = "CompBot Swerve", group = "CompBot")
public class CompBot extends LinearOpMode {

    DcMotor FLMotor, BLMotor, BRMotor, FRMotor, pivot, slide;

    Servo FLServo, BLServo, BRServo, FRServo, claw;

    GoBildaPinpointDriver odo;


    // Main swerve stuff is here
    Swerve_components swerve;

    // Main mechanism stuff is here
    Mekanism mec;


    ElapsedTime turnTime = new ElapsedTime();


    // In case builders are bad, is offset center for servo
    double FLServoOffSet = .005;
    double FRServoOffSet = .00;
    double BLServoOffSet = .01;
    double BRServoOffSet = 0.007;


    static double TRACKWIDTH = 14; //in inches
    static double WHEELBASE = 15; //in inches


    /**
     * controls for game pad 1:
     * right trigger: forwards
     * left trigger: backwards
     * right stick x: rotate
     * left stick x: strafe
     * <p>
     * controls for game pad 2:
     * left stick y: in and out of arm
     * right stick y: up and down of arm
     * left trigger: claw intake
     * right trigger: claw out
     * presets for:
     * attaching clip to sample
     * attaching specimen(clip + sample) to top rung
     * presets for bucket 1 and 2
     */
    public void runOpMode() throws InterruptedException {

        swerve.initRobot(); // Inits the swerve components
        mec.initRobot(); // Inits the mechanism components


        waitForStart();
        while (opModeIsActive()) {

            //game pad 1
            double speedGMP1 = gamepad1.left_trigger - gamepad1.right_trigger; // Makes it so that the triggers cancel each other out if both are pulled at the same time
            double angleGMP1 = -gamepad1.right_stick_x;

            if (speedGMP1 != 0) move(gamepad1.left_stick_x, speedGMP1);
            else if (angleGMP1 != 0) rotate(angleGMP1);
            else {
                FLMotor.setPower(0);
                BLMotor.setPower(0);
                BRMotor.setPower(0);
                FRMotor.setPower(0);
            }
            // if (gamepad1.a) rotateToCenter();


            //game pad 2
            double armLength = -gamepad2.right_stick_y;
            double armAngle = -gamepad2.left_stick_y;

            // 0 < arm length < 4268
            if (armLength > 0 && slide.getCurrentPosition() > 4268) {
                armLength = 0;
            } else if (armLength < 0 && slide.getCurrentPosition() < -4268) {
                armLength = 0;
            }
            slide.setPower(armLength);

            // 0 < arm angle < 2904
            if (armAngle > 0 && pivot.getCurrentPosition() > 2904) {
                armAngle = 0;
            } else if (armAngle < 0 && pivot.getCurrentPosition() < -2904) {
                armAngle = 0;
            }
            pivot.setPower(armAngle);

            //to test arm length and angle
            addTelem(pivot.getCurrentPosition(), slide.getCurrentPosition(), armAngle, armLength);
        }
    }

    public void addTelem(int x, int y, double a, double b) {
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
     * Moves the robot based on desired heading and power.<br>
     * Does not change the rotation of the robot at all
     *
     * @param heading Desired heading of the robot.<br>
     *                0 is straight forward 1 is fully right, -1 is fully left
     * @param power   Desired power to run the motors at
     */
    public void move(double heading, double power) {
        heading = (heading + 1) / 2;

        FLServo.setPosition(heading + FLServoOffSet);
        BLServo.setPosition(heading + BLServoOffSet);
        BRServo.setPosition(heading + BRServoOffSet);
        FRServo.setPosition(heading + FRServoOffSet);

        FLMotor.setPower(power);
        BLMotor.setPower(power);
        BRMotor.setPower(power);
        FRMotor.setPower(power);
    }


    /**
     * Rotates the robot around a center point.<br>
     * Does not move the robot in any other direction.<br>
     *
     * @param power Power to turn the robot at
     */
    public void rotate(double power) {

        // Set wheels for rotation (Ben's robot has 2x gear ratio so .25/2 and .75/2)
        FLServo.setPosition(.25 + .125 / 2);
        BLServo.setPosition(.75 - .125 / 2);
        BRServo.setPosition(.25 + .125 / 2);
        FRServo.setPosition(.75 - .125 / 2);

        //turn motors to rotate robot
        FLMotor.setPower(-power);
        BLMotor.setPower(-power);
        BRMotor.setPower(power);
        FRMotor.setPower(power);

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

        double i_WheelAngle = Math.atan2(WHEELBASE, turnAmount - TRACKWIDTH / 2);
        double outsideAng = Math.atan2(WHEELBASE, turnAmount + TRACKWIDTH / 2);


    }


    /**
     * TODO Possibly make this not run at full speed at all times by adding some sort of power input
     * Uses the IMU to move the robot to face forward
     * <p>
     * As long as this function is called, it will try to rotate back to facing forward
     */
    public void rotateToCenter() {
        double orientation = odo.getHeading();
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
