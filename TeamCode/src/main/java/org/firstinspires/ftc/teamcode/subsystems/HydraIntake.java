package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.objects.HydraOpMode;

public class HydraIntake {
    private final DcMotor mMotPxlIntk;
    private final double mPwrIn;
    private final double mPwrOut;
    private HydraOpMode mOp;

    public HydraIntake(HydraOpMode op, String motor, double pwrIn, double pwrOut) {
        mOp = op;
        // initialize the motor object
        mMotPxlIntk = mOp.mHardwareMap.get(DcMotor.class, motor);
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
