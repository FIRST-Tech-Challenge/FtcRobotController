package org.firstinspires.ftc.teamcode.CompBot;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class MekanismConfig {

    public LinearOpMode myOp;

    public MekanismConfig(LinearOpMode opMode) {
        myOp = opMode;
    }

    DcMotor pivot, slide;
    Servo claw, wrist, spintake;


    int ARM_COUNTS_PER_INCH = 250; // Encoder counts per inch of slide movement
    int ARM_COUNTS_PER_DEGREE = 20; // Encoder count per degree of slide angle


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

        pivot.setDirection(DcMotor.Direction.FORWARD);
        slide.setDirection(DcMotor.Direction.FORWARD);

        pivot.setMode(STOP_AND_RESET_ENCODER);
        slide.setMode(STOP_AND_RESET_ENCODER);

        pivot.setMode(RUN_USING_ENCODER);
        slide.setMode(RUN_USING_ENCODER);

        pivot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Sets maximum allowed power to 1
        pivot.setPower(1);
        slide.setPower(1);


        wrist = myOp.hardwareMap.get(Servo.class, "wrist");
        claw = myOp.hardwareMap.get(Servo.class, "claw");
        claw.scaleRange(0, 1);


    }


    /**
     * Sets the length of the arm
     * <p>
     * In inches.
     *
     * @param length Length of the arm
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
    public void moveClaw(double position) {

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
}
