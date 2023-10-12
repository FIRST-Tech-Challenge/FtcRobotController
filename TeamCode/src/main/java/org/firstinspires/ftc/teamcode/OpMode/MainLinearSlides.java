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
        leftSlide.setPower(-rightTrigger);
        rightSlide.setPower(rightTrigger);
        leftSlide.setPower(leftTrigger);
        rightSlide.setPower(-leftTrigger);
    }

    public static void moveToLowerUpper(boolean leftBumper, boolean rightBumper) {

        if(leftBumper) {
            if(leftSlide.getCurrentPosition() > 0 && rightSlide.getCurrentPosition() > 0) {
                leftSlide.setTargetPosition(0);
                rightSlide.setTargetPosition(0);

                leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                leftSlide.setPower(1);
                rightSlide.setPower(-1);
            } else {
                leftSlide.setPower(0);
                rightSlide.setPower(0);
            }
        } else if(rightBumper) {
            if(leftSlide.getCurrentPosition() < 3200 && rightSlide.getCurrentPosition() < 3200) {
                leftSlide.setTargetPosition(3200);
                rightSlide.setTargetPosition(3200);

                leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                leftSlide.setPower(-1);
                rightSlide.setPower(1);
            } else {
                leftSlide.setPower(0);
                rightSlide.setPower(0);
            }
        }

    }
}
