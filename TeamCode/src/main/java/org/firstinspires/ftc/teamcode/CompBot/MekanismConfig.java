package org.firstinspires.ftc.teamcode.CompBot;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_TO_POSITION;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/*

Pivot is the pivot for the arm.

Slide is the actual slide in and out

 */

public class MekanismConfig {

    public LinearOpMode myOp;

    public MekanismConfig(LinearOpMode opMode) {
        myOp = opMode;
    }

    ElapsedTime homeTimer = new ElapsedTime();

    DcMotor pivot, slide;
    Servo claw, wrist, spintake;

    DigitalChannel pivotHome, slideHome;


    static int ARM_COUNTS_PER_INCH = 250; // Encoder counts per inch of slide movement
    static int ARM_COUNTS_PER_DEGREE = 20; // Encoder count per degree of slide angle


    /**
     * Initializes the mechanism of the robot.
     * <p>
     * Starts all the devices and maps where they go,
     * as well as sets direction and whether motors run with encoders or not.
     */
    public void initMekanism() {

        // Init slaw, claw, and pivot
        pivot = myOp.hardwareMap.get(DcMotor.class, "pivot");
        slide = myOp.hardwareMap.get(DcMotor.class, "slide");

        pivot.setTargetPosition(0);
        slide.setTargetPosition(0);

        // TODO: Find the correct direction for these
        pivot.setDirection(DcMotor.Direction.FORWARD);
        slide.setDirection(DcMotor.Direction.FORWARD);

        pivot.setMode(STOP_AND_RESET_ENCODER);
        slide.setMode(STOP_AND_RESET_ENCODER);

        pivot.setMode(RUN_TO_POSITION);
        slide.setMode(RUN_TO_POSITION);

        pivot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        pivot.setPower(1.0);
        slide.setPower(1.0);


        // Servo Configs
        wrist = myOp.hardwareMap.get(Servo.class, "wrist");
        claw = myOp.hardwareMap.get(Servo.class, "claw");
        spintake = myOp.hardwareMap.get(Servo.class, "spintake");

        // Sets the end stops of the servos
        wrist.scaleRange(0, 1);
        claw.scaleRange(0, 1);
        spintake.scaleRange(0, 1);


        // Homing switches config
        pivotHome = myOp.hardwareMap.get(DigitalChannel.class, "pivotHome");
        slideHome = myOp.hardwareMap.get(DigitalChannel.class, "slideHome");

        pivotHome.setMode(DigitalChannel.Mode.INPUT);
        slideHome.setMode(DigitalChannel.Mode.INPUT);
    }


    /**
     * Homes the pivot.
     * <p>
     * Runs the pivot up until either the limit switch is triggered
     * or the program is stopped
     *
     * @see #homeSlide()
     */
    public void homePivot() {
        pivot.setMode(RUN_USING_ENCODER);

        // Goes until the switch is pressed or program is stopped
        homeTimer.reset();
        while (myOp.opModeIsActive() && !pivotHome.getState() && homeTimer.seconds() < 3)
            pivot.setPower(0.3);
        pivot.setPower(0);

        // Resets the encoder
        pivot.setMode(STOP_AND_RESET_ENCODER);
        pivot.setMode(RUN_TO_POSITION);

        pivot.setPower(1); // Sets the maximum allowed power
    }


    /**
     * Homes the slide.
     * <p>
     * Runs the slide in until either the limit switch is triggered
     * or the program is stopped
     *
     * @see #homePivot()
     */
    public void homeSlide() {

        slide.setMode(RUN_USING_ENCODER);

        // Goes until the switch is pressed or program is stopped
        homeTimer.reset();
        while (myOp.opModeIsActive() && !slideHome.getState() && homeTimer.seconds() < 3) {
            slide.setPower(0.3);
        }

        slide.setPower(0);

        // Resets the encoder
        slide.setMode(STOP_AND_RESET_ENCODER);
        slide.setMode(RUN_TO_POSITION);

        slide.setPower(1); // Sets the maximum allowed power
    }


    /**
     * Sets the length of the arm
     * <p>
     * In inches.
     *
     * @param length Length of the arm
     * @see #armAngle(double)
     */
    public void armLength(double length) {

        slide.setTargetPosition((int) (length * ARM_COUNTS_PER_INCH));
    }


    /**
     * Sets the angle of the arm
     * <p>
     * In degrees.
     *
     * @param angle Angle of the arm
     * @see #armLength(double)
     */
    public void armAngle(double angle) {
        pivot.setTargetPosition((int) (angle * ARM_COUNTS_PER_DEGREE));
    }


    /**
     * Sets the length and angle independently
     * <p>
     * In inches and degrees
     *
     * @param length Length of the arm
     * @param angle  Angle of the arm
     */
    public void basicMoveArm(double length, double angle) {

        slide.setTargetPosition((int) (length * ARM_COUNTS_PER_INCH));
        pivot.setTargetPosition((int) (angle * ARM_COUNTS_PER_DEGREE));
    }


    /**
     * Sets the position of the arm by the XY coordinates of the end.
     * <p>
     * All in Inches.
     * <p>
     * Max 20IN for both X and Y coordinates.
     *
     * @param x X coordinate of the arm
     * @param y Y coordinate of the arm
     */
    public void moveXY(double x, double y) {

        // Binds x to a valid coordinate
        if (x < 0) {
            x = 0;
        } else if (x > 20) {
            x = 20;
        }

        // Binds y to a valid coordinate
        if (y < 0) {
            y = 0;
        } else if (y > 20) {
            y = 20;
        }

        double armLen = Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2)); // Calculates arm length
        double armAngle = Math.atan(y / x); // Calculates arm angle

        slide.setTargetPosition((int) (armLen * ARM_COUNTS_PER_INCH));
        pivot.setTargetPosition((int) (armAngle * ARM_COUNTS_PER_DEGREE));
    }


    /**
     * Sets the power of the claw
     *
     * @param position (-1) to 1.
     *                 <p> 1 - Full intake.
     *                 <p> (-1) - Full reverse
     */
    public void setClaw(double position) {

        position = (position + 1) / 2;

        claw.setPosition(position);
    }


    /**
     * Sets the power of the intake
     *
     * @param power (-1) to 1.
     *              <p> 1 - Full intake.
     *              <p> (-1) - Full reverse
     */
    public void setSpintake(double power) {

        power = (power + 1) / 2;

        spintake.setPosition(power);
    }


    /**
     * Pauses slide at the current position
     *
     * @see #pausePivot()
     */
    public void pauseSlide() {
        slide.setTargetPosition(slide.getCurrentPosition());
    }

    /**
     * Pauses pivot at the current position
     *
     * @see #pauseSlide()
     */
    public void pausePivot() {
        pivot.setTargetPosition(pivot.getCurrentPosition());
    }

}
