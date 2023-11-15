package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion;

public class HydraIntake extends BlocksOpModeCompanion {
    private final DcMotor mMotPxlIntk;
    private final double mPwrIn;
    private final double mPwrOut;

    public HydraIntake(String motor, double pwrIn, double pwrOut) {
        // initialize the motor object
        mMotPxlIntk = hardwareMap.get(DcMotor.class, motor);
        // power values
        mPwrIn = pwrIn;
        mPwrOut = pwrOut;
    }

    /**
     * Start running the intake to bring pixels in
     */
    public void StartIn() {
        mMotPxlIntk.setPower(mPwrIn);
    }

    /**
     * Start running the intake to push pixels out
     */
    public void StartOut() {
        mMotPxlIntk.setPower(mPwrOut);
    }

    /**
     * Stop running the intake
     */
    public void Stop() {
        mMotPxlIntk.setPower(0);
    }
}
