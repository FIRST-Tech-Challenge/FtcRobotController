package org.firstinspires.ftc.teamcode.utility;

import static java.lang.Math.abs;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class LinearSlideMovement {

    private IntakeMovement intake;

    private DcMotorEx leftLinearSlide;

    private DcMotorEx rightLinearSlide;

    public static final int top_linearslide_ticks = 1660; // adjusted to proper field height at T1 from memory

    public static final int mid_linearslide_ticks = 1000;

    public static final int low_linearslide_ticks = 600;

    public static final int bottom_linearslide_ticks = 0;

    /**
     * Pulls in information about the motors that is determined during initialization and the
     * IntakeMovement class and makes that information accessible to the rest of this class
     * @param leftSlide  the left slide motor
     * @param  rightSlide  the right slide motor
     * @param  intakeMovement  the IntakeMovement class
     */
    public LinearSlideMovement(DcMotorEx leftSlide, DcMotorEx rightSlide, IntakeMovement intakeMovement){
        leftLinearSlide = leftSlide;
        rightLinearSlide = rightSlide;
        intake = intakeMovement;
        initMovement();
    }

    /**
     * Resets all wheel motor encoder positions to 0
     */
    private void initMovement(){
        leftLinearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightLinearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void LinearSlidesTop(){
        leftLinearSlide.setVelocityPIDFCoefficients(10.0, 0.2, 0.001, 10.0);
        rightLinearSlide.setVelocityPIDFCoefficients(10.0, 0.2, 0.001, 10.0);
        Movelinearslide (top_linearslide_ticks);
    }

    public void LinearSlidesMiddle(){
        leftLinearSlide.setVelocityPIDFCoefficients(10.0, 0.2, 0.001, 10.0);
        rightLinearSlide.setVelocityPIDFCoefficients(10.0, 0.2, 0.001, 10.0);
        Movelinearslide (mid_linearslide_ticks);
    }


    public void LinearSlidesLow(){
        leftLinearSlide.setVelocityPIDFCoefficients(10.0, 0.2, 0.001, 10.0);
        rightLinearSlide.setVelocityPIDFCoefficients(10.0, 0.2, 0.001, 10.0);
        Movelinearslide (low_linearslide_ticks);
    }

    public void LinearSlidesBottom(){
        leftLinearSlide.setVelocityPIDFCoefficients(15.0, 0.5, 0.001, 2.0);
        rightLinearSlide.setVelocityPIDFCoefficients(15.0, 0.5, 0.001, 2.0);
        Movelinearslide (bottom_linearslide_ticks);
        while (leftLinearSlide.getCurrentPosition() > (LinearSlideMovement.bottom_linearslide_ticks + 13)){
            // pause to wait for the slide to lower before raising the wrist back up.
        }
        intake.FlipUp();
    }

    public void Movelinearslide(int ticks){
        intake.FlipSafety();
        leftLinearSlide.setTargetPosition(ticks);
        rightLinearSlide.setTargetPosition(ticks);
        leftLinearSlide.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        rightLinearSlide.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        leftLinearSlide.setPower(1);
        rightLinearSlide.setPower(1);
    }
}
