package org.firstinspires.ftc.teamcode.Modules;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

// do you like my comments?
// ----- READY TO TRANSFER ----- //

public class Slides {

    ///////////////////////////////////////////////
    ////                                     /////
    ////              VARIABLES              /////
    ////                                     /////
    //////////////////////////////////////////////

    // ---------- SLIDE MOTORS ---------- //
    private final DcMotor lSlide, rSlide;

    // -------- SLIDE PID STUFF -------- //
    private double slideKP = 0.02; /*Unknown*/
    private double slideKI = 0.055; /*Unknown*/
    private double slideKD = 0.00005; /*Unknown*/
    private double slideKF = 0.01; /*Unknown feedforward term, used to counteract gravity*/
    private PIDController slidePID = new PIDController(slideKP, slideKI, slideKD);
    private double slideGoal = 0;

    ///////////////////////////////////////////////
    ////                                     /////
    ////              FUNCTIONS              /////
    ////                                     /////
    //////////////////////////////////////////////

    // --------- INITIALIZATION --------- //

    public Slides(HardwareMap hardwareMap) {

        rSlide = hardwareMap.dcMotor.get("rSlide");
        lSlide = hardwareMap.dcMotor.get("lSlide");

        slidePID.setDeadZone(0);

        lSlide.setDirection(DcMotor.Direction.REVERSE);
        rSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        slidePID.setPID(slideKP,slideKI, slideKD);
    }

    // ------------ GETTERS ------------ //

    public double getSlideGoalPosition() {return slideGoal;}

    public double getSlidePositionAvg(){
        return (double) (rSlide.getCurrentPosition() + lSlide.getCurrentPosition()) /2;
    }

    /**
     * Only gets the right slide position to avoid any weird shenanigans with
     * tuning a PID of a combined encoder output
     */
    public double getOneSlidePosition(){
        return rSlide.getCurrentPosition();
    }

    // ------------ SETTERS ------------ //

    public void setSlidePIDConstants(double ki, double kp, double kd){
        slideKI = ki;
        slideKP = kp;
        slideKD = kd;
    }

    public void setSlidePower(double power) {
        rSlide.setPower(power);
        lSlide.setPower(power);
    }

    /**
     * This version of setSlidePower is intended to be used with setSlideGoalPosition.
     * First, use setSlideGoalPosition within the main loop/controller, and afterwards,
     * at the very end of each iteration/loop, use this to update slide powers.
     */
    public void setSlidePower(){
        setSlidePower(slidePID.calculate(getOneSlidePosition(), slideGoal) + slideKF);
    }

    public void setSlideGoalPosition(double goal){

        if(this.slideGoal != goal){
            slidePID.resetIntegral();
        }

        this.slideGoal = goal;
    }

    public void resetSlideEncoders(){
        rSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lSlide.setDirection(DcMotorSimple.Direction.REVERSE);
    }
}