package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion;

public class HydraIntake extends BlocksOpModeCompanion {
    private final DcMotor mMotPxlIntk;
    private final double mPwrIn;
    private final double mPwrOut;

    public HydraIntake(String motor, double pwrIn, double pwrOut) {
        mMotPxlIntk = hardwareMap.get(DcMotor.class, motor);
        mPwrIn = pwrIn;
        mPwrOut = pwrOut;
    }
    public void StartIn() {
        mMotPxlIntk.setPower(mPwrIn);
    }

    public void StartOut() {
        mMotPxlIntk.setPower(mPwrOut);
    }

    public void Stop() {
        mMotPxlIntk.setPower(0);
    }
}
