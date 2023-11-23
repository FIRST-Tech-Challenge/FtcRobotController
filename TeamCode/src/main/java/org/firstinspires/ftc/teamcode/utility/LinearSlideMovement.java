package org.firstinspires.ftc.teamcode.utility;

import com.qualcomm.robotcore.hardware.DcMotor;

public class LinearSlideMovement {

    static final int top_linearslide_ticks = 2000;

    static final int mid_linearslide_ticks = 1000;

    static final int low_linearslide_ticks = 500;

    static final int bottom_linearslide_ticks = 0;

    private DcMotor leftLinearSlide;

    private DcMotor rightLinearSlide;

    public void LinearSlideMovement(DcMotor leftSlide, DcMotor rightSlide){
        leftLinearSlide = leftSlide;
        rightLinearSlide = rightSlide;
    }

    private void initMovement(){
        leftLinearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightLinearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void linearslidetop(){

    }
}
