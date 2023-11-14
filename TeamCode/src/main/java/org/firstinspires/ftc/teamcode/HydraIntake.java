package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion;

public class HydraIntake extends BlocksOpModeCompanion {
    private DcMotor mMotPxlIntk;
    private double mPwrIn;
    private double mPwrOut;

    public void Init(String motor, double pwrIn, double pwrOut) {
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
