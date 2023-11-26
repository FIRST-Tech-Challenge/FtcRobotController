package org.firstinspires.ftc.teamcode.utility;

import com.qualcomm.robotcore.hardware.DcMotor;

public class LinearSlideMovement {

    private IntakeMovement intake;

    private DcMotor leftLinearSlide;

    private DcMotor rightLinearSlide;

    /**
     * Pulls in information about the motors that is determined during initialization and the
     * IntakeMovement class and makes that information accessible to the rest of this class
     * @param leftSlide  the left slide motor
     * @param  rightSlide  the right slide motor
     * @param  intakeMovement  the IntakeMovement class
     */
    public LinearSlideMovement(DcMotor leftSlide, DcMotor rightSlide, IntakeMovement intakeMovement){
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

    public void Movelinearslide(int ticks){
        intake.intakeIsSafe = false;
        intake.FlipDown();
        if (intake.setSafety() == true){
            leftLinearSlide.setTargetPosition(ticks);
            rightLinearSlide.setTargetPosition(ticks);
            leftLinearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightLinearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftLinearSlide.setPower(0.8);
            rightLinearSlide.setPower(0.8);
        }
    }
}
