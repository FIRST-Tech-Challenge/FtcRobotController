package com.kalipsorobotics.fresh.mechanism;

import com.kalipsorobotics.fresh.OpModeUtilities;
import com.qualcomm.robotcore.hardware.DcMotor;

public class OuttakeSlide {

    DcMotor linearSlide;
    OpModeUtilities opModeUtilities;

    private OuttakeSlide(DcMotor linearSlide) {
        this.linearSlide = linearSlide;
    }
    public OuttakeSlide(OpModeUtilities opModeUtilities) {
        this(opModeUtilities.getHardwareMap().dcMotor.get("linearSlide"));
    }

    public void setUp() {
        linearSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        linearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void run() {
        linearSlide.setPower(1);
    }

    public void reverse() {
        linearSlide.setPower(-1);
    }

    public void stop() {
        linearSlide.setPower(0);
    }

}
