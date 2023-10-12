package org.firstinspires.ftc.teamcode.OpMode;

import com.qualcomm.robotcore.hardware.DcMotor;

public class MainLinearSlides {

    static DcMotor leftSlide;
    static DcMotor rightSlide;

    public static void linearSlideInit(DcMotor left, DcMotor right) {
        leftSlide = left;
        rightSlide = right;
    }

    public static void manualMove(float leftTrigger, float rightTrigger) {
        leftSlide.setPower(rightTrigger);
        rightSlide.setPower(rightTrigger);
        leftSlide.setPower(-leftTrigger);
        rightSlide.setPower(-leftTrigger);
    }
}
