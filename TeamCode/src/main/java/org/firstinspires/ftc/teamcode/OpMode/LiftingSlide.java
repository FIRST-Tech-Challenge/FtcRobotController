package org.firstinspires.ftc.teamcode.OpMode;

import com.qualcomm.robotcore.hardware.DcMotor;

public class LiftingSlide {

    static DcMotor slideMotor;
    static boolean isDown = true;

    public static void liftingSlideInit(DcMotor slide) {
        slideMotor = slide;
    }

    public static void hoist(boolean start) {
        if(start) {
            slideMotor.setTargetPosition(isDown ? 1600 : 0);
            slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //Do i need to turn off power? test this; runToPos may automatically stop motor once targetPos is reached.
        }
    }

}
