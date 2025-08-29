package org.firstinspires.ftc.teamcode.subsystems.mecanum;

import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.subsystems.odometry.PinPointOdometrySubsystem;
import org.firstinspires.ftc.teamcode.util.pidcore.PIDCore;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Command wrapper for controlling a Mecanum drive system using
 * positional PID and field-oriented driving. This class consolidates
 * subsystem interactions and provides high-level movement commands.
 */
public class MecanumCommand {


    // PID controllers for x, y, and heading
    private PIDCore xController;
    private PIDCore yController;
    private PIDCore thetaController;

    // create a class to consolidate subsystems
    private MecanumSubsystem mecanumSubsystem;
    private PinPointOdometrySubsystem pinPointOdoSubsystem;

    // hardware is owned by test and pass down to subsystems
    private Hardware hw;
    private boolean runMotionProfile;

    private LinearOpMode opMode;
    private ElapsedTime elapsedTime;
    ;

    private double ex = 0;
    private double ey = 0;
    private double etheta = 0;
    public double xFinal, yFinal, thetaFinal;
    public double velocity;


    /**
     * Creates a new MecanumCommand instance.
     *
     * @param opmode The active {@link LinearOpMode} controlling the robot.
     * @param hw     Hardware configuration object for accessing devices.
     */
    public MecanumCommand(LinearOpMode opmode, Hardware hw) {
        this.hw = hw;
        this.mecanumSubsystem = new MecanumSubsystem(hw);
        this.pinPointOdoSubsystem = new PinPointOdometrySubsystem(hw);
        this.opMode = opmode;
        elapsedTime = new ElapsedTime();
        xFinal = pinPointOdoSubsystem.getX();
        yFinal = pinPointOdoSubsystem.getY();
        thetaFinal = pinPointOdoSubsystem.getHeading();
        velocity = 0;
        turnOffInternalPID();
    }

    /**
     * Updates PID constants for X, Y, and theta control loops.
     *
     * @param kpx     Proportional gain for X-axis PID.
     * @param kdx     Derivative gain for X-axis PID.
     * @param kix     Integral gain for X-axis PID.
     * @param kpy     Proportional gain for Y-axis PID.
     * @param kdy     Derivative gain for Y-axis PID.
     * @param kiy     Integral gain for Y-axis PID.
     * @param kptheta Proportional gain for theta (heading) PID.
     * @param kdtheta Derivative gain for theta PID.
     * @param kitheta Integral gain for theta PID.
     */
    public void setConstants(double kpx, double kdx, double kix, double kpy, double kdy, double kiy, double kptheta, double kdtheta, double kitheta) {
        MecanumConstants.kpx = kpx;
        MecanumConstants.kdx = kdx;
        MecanumConstants.kix = kix;
        MecanumConstants.kpy = kpy;
        MecanumConstants.kdy = kdy;
        MecanumConstants.kiy = kiy;
        MecanumConstants.kptheta = kptheta;
        MecanumConstants.kdtheta = kdtheta;
        MecanumConstants.kitheta = kitheta;
        mecanumSubsystem.updatePIDConstants();
    }

    /**
     * Disables the internal PID control in the mecanum subsystem.
     * Useful if external PID control or direct power control is preferred.
     */
    public void turnOffInternalPID() {
        mecanumSubsystem.turnOffInternalPID();
    }


    /**
     * Runs PID control using PinPoint odometry to drive toward the target
     * {@code xFinal}, {@code yFinal}, and {@code thetaFinal} positions.
     * Limits motion based on the configured {@code velocity}.
     */
    public void processPIDUsingPinpoint() {
        ex = mecanumSubsystem.globalXControllerOutputPositional(xFinal, pinPointOdoSubsystem.getX());
        ey = mecanumSubsystem.globalYControllerOutputPositional(yFinal, pinPointOdoSubsystem.getY());
        etheta = mecanumSubsystem.globalThetaControllerOutputPositional(thetaFinal, pinPointOdoSubsystem.getHeading());


        double max = Math.max(Math.abs(ex), Math.abs(ey));
        if (max > velocity) {
            double scalar = velocity / max;
            ex *= scalar;
            ey *= scalar;
            etheta *= scalar;
        }

        moveGlobalPartialPinPoint(true, ex, ey, etheta);

    }

    /**
     * Wrapper for field-oriented movement for TeleOp modes using PinPoint heading.
     *
     * @param vertical   Forward/backward input (-1 to 1).
     * @param horizontal Left/right strafe input (-1 to 1).
     * @param rotational Rotation input (-1 to 1).
     */
    public void fieldOrientedMove(double vertical, double horizontal, double rotational) {
        mecanumSubsystem.fieldOrientedMove(vertical, horizontal, rotational, pinPointOdoSubsystem.getHeading());
    }


    /**
     * Moves the robot in global coordinates using partial control (drive + rotation).
     * Converts global X/Y commands to local robot-oriented movement based on heading.
     *
     * @param run        If true, execute movement; if false, stop.
     * @param vertical   Global Y-axis movement (forward/back).
     * @param horizontal Global X-axis movement (strafe).
     * @param rotational Rotation command.
     */
    public void moveGlobalPartialPinPoint(boolean run, double vertical, double horizontal, double rotational) {
        if (run) {
            //might have to change this because Gobilda Odommetry strafing left is POSITIVE while this works for strafing right is Positive
            double angle = Math.PI / 2 - pinPointOdoSubsystem.getHeading();
            double localVertical = vertical * Math.cos(pinPointOdoSubsystem.getHeading()) - horizontal * Math.cos(angle);
            double localHorizontal = vertical * Math.sin(pinPointOdoSubsystem.getHeading()) + horizontal * Math.sin(angle);
            mecanumSubsystem.partialMove(true, localVertical, localHorizontal, rotational);
        }
    }

    public void resetPinPointOdometry() {
        pinPointOdoSubsystem.reset();
    }

    public boolean moveToPos(double x, double y, double theta) {
        elapsedTime.reset();
        setFinalPositionMotionProfile(true, 30, x, y, theta, false);
        return positionNotReachedYet();
    }

    public void setFinalPositionMotionProfile(boolean run, double velocity, double x, double y, double theta, boolean runMotionProfile) {
        if (run) {
            this.xFinal = x;
            this.yFinal = y;
            this.thetaFinal = theta;
            this.velocity = velocity;
            this.runMotionProfile = runMotionProfile;
        }
    }

    public boolean positionNotReachedYet() {
        return !(isXReached() && isYReached() && isThetaReached());
    }

    public double getXDifferencePinPoint() {
        return Math.abs(this.xFinal - pinPointOdoSubsystem.getX());
    }

    public double getYDifferencePinPoint() {
        return Math.abs(this.yFinal - pinPointOdoSubsystem.getY());
    }

    public double getThetaDifferencePinPoint() {
        return Math.abs(this.thetaFinal - pinPointOdoSubsystem.getHeading());
    }

    public boolean isYReached() {
        return getYDifferencePinPoint() < 2.5;
    }

    public boolean isXReached() {
        return getXDifferencePinPoint() < 2.5;
    }

    public boolean isThetaReached() {
        return getThetaDifferencePinPoint() < 0.07;
    }

    public double getOdoX(){
        return pinPointOdoSubsystem.getX();
    }
    public double getOdoY(){
        return pinPointOdoSubsystem.getY();
    }
    public double getOdoHeading(){
        return pinPointOdoSubsystem.getHeading();
    }
    public boolean isThetaPassed(){
        return getThetaDifferencePinPoint() < 0.22;
    }

    public boolean isXPassed(){
        return getXDifferencePinPoint() < 10;
    }
    public boolean isYPassed(){
        return getYDifferencePinPoint() < 10;
    }


    //teleop

    public void handleMovement(double leftStickY, double leftStickX, double rightStickX) {

        fieldOrientedMove(-leftStickY, leftStickX, rightStickX);
    }



}

