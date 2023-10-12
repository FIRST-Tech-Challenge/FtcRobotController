package org.firstinspires.ftc.teamcode.OpMode;

import com.qualcomm.robotcore.hardware.DcMotor;

public class LiftingSlide {

    static DcMotor slideMotor;

    public static void liftingSlideInit(DcMotor slide) {
        slideMotor = slide;
    }

}
