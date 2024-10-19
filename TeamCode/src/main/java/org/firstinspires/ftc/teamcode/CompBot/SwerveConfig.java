package org.firstinspires.ftc.teamcode.CompBot;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.ODO.GoBildaPinpointDriver;
import org.opencv.core.Mat;

public class SwerveConfig {

    public LinearOpMode myOp;

    SwerveConfig(LinearOpMode opMode) {
        myOp = opMode;
    }

    GoBildaPinpointDriver odo;


    DcMotor FLMotor, BLMotor, FRMotor, BRMotor, pivot, slide;


    Servo FLServo, BLServo, FRServo, BRServo, claw;

    /**
     * TODO Get rid of this somehow
     * <p>
     * all of these need to be removed somehow
     */
    // In case builders are bad, is offset center for servo
    double FLServoOffSet = .005;
    double BLServoOffSet = .01;
    double FRServoOffSet = .00;
    double BRServoOffSet = 0.007;

    static double TRACKWIDTH = 14, WHEELBASE = 15;


    public void initSwerve() {

        // Maps the motor objects to the physical ports
        FLMotor = hardwareMap.get(DcMotor.class, "FLMotor");
        BLMotor = hardwareMap.get(DcMotor.class, "BLMotor");
        FRMotor = hardwareMap.get(DcMotor.class, "FRMotor");
        BRMotor = hardwareMap.get(DcMotor.class, "BRMotor");

        // Sets the encoder mode
        FLMotor.setMode(RUN_USING_ENCODER);
        BLMotor.setMode(RUN_USING_ENCODER);
        FRMotor.setMode(RUN_USING_ENCODER);
        BRMotor.setMode(RUN_USING_ENCODER);

        // Sets what happens when no power is applied to the motors.
        // In this mode, the computer will short the 2 leads of the motor, and because of math, the motor will be a lot harder to turn
        FLMotor.setZeroPowerBehavior(BRAKE);
        BLMotor.setZeroPowerBehavior(BRAKE);
        FRMotor.setZeroPowerBehavior(BRAKE);
        BRMotor.setZeroPowerBehavior(BRAKE);

        FLMotor.setDirection(REVERSE);
        BLMotor.setDirection(REVERSE);
        FRMotor.setDirection(REVERSE);
        BRMotor.setDirection(FORWARD);


        // Maps the servo objects to the physical ports
        FLServo = hardwareMap.get(Servo.class, "FLServo");
        BLServo = hardwareMap.get(Servo.class, "BLServo");
        FRServo = hardwareMap.get(Servo.class, "FRServo");
        BRServo = hardwareMap.get(Servo.class, "BRServo");

        // Sets the ends of the servos. Hover cursor over function for more info
        // Will need to be tuned later
        FLServo.scaleRange(FLServoOffSet, 1.0 + FLServoOffSet * 2);
        BLServo.scaleRange(BLServoOffSet, 1.0 + BLServoOffSet * 2);
        FRServo.scaleRange(FRServoOffSet, 1.0 + FRServoOffSet * 2);
        BRServo.scaleRange(BRServoOffSet, 1.0 + BRServoOffSet * 2);

        FLServo.setPosition(0.50 + FLServoOffSet);
        BLServo.setPosition(0.51 + BLServoOffSet);
        FRServo.setPosition(0.50 + FRServoOffSet);
        BRServo.setPosition(0.51 + BRServoOffSet);


        // Init GoBilda Pinpoint module
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        odo.resetPosAndIMU();
        odo.setOffsets(177.8, 50.8);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
    }


    /**
     * Moves the robot based on desired heading and power.
     * <p>
     * Only moves in a straight line.
     * Does not change the rotation of the robot at all
     *
     * @param heading Desired heading of the robot.<br>
     *                0 is straight forward 1 is fully right, -1 is fully left
     * @param power   Desired power to run the motors at
     */
    public void moveStraight(double heading, double power) {
        heading = (heading + 1) / 2; // Converts the -1 to 1 input range to 0 to 1 for the servos


        FLMotor.setPower(power);
        BLMotor.setPower(power);
        BRMotor.setPower(power);
        FRMotor.setPower(power);

        FLServo.setPosition(heading + FLServoOffSet);
        BLServo.setPosition(heading + BLServoOffSet);
        BRServo.setPosition(heading + BRServoOffSet);
        FRServo.setPosition(heading + FRServoOffSet);
    }


    /**
     * Rotates the robot around a center point.<br>
     * Does not move the robot in any other direction.<br>
     *
     * @param power Power to turn the robot at
     */
    public void rotate(double power) {

        //turn motors to rotate robot
        FLMotor.setPower(-power);
        BLMotor.setPower(-power);
        BRMotor.setPower(power);
        FRMotor.setPower(power);


        // Set wheels for rotation (Ben's robot has 2x gear ratio so .25/2 and .75/2)
        FLServo.setPosition(.25 + .125 / 2);
        BLServo.setPosition(.75 - .125 / 2);
        BRServo.setPosition(.25 + .125 / 2);
        FRServo.setPosition(.75 - .125 / 2);

    }


    /**
     * TODO this needs to be worked on nothing is done here
     *
     * <p>
     * Wheel angle is perpendicular to turn angle
     * turn angle is inverse tan(get angle) of 7.5(half of wheel base length) / (turning distance/2)
     * because it is radius of point we are trying to rotate around
     * speed = -1 to 1
     * turnRad = -1 to 1
     * turnDir = LEFT or RIGHT
     */
    public void moveAndRotate(double power, double turn) {

        double i_WheelAngle = Math.atan2(WHEELBASE, turn - TRACKWIDTH / 2);
        double outsideAng = Math.atan2(WHEELBASE, turn + TRACKWIDTH / 2);

    }


    /**
     * TODO Possibly make this not run at full speed at all times by adding some sort of power input
     * Uses the IMU to move the robot to face forward
     * <p>
     * As long as this function is called, it will try to rotate back to facing forward
     */
    public void centerRobot() {
        double orientation = odo.getHeading();

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
