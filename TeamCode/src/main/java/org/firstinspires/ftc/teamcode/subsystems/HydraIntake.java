package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.objects.HydraOpMode;

public class HydraIntake {
    private final DcMotor mMotPxlIntk;
    final String cfgIntakeMotor = "MotPxlIntk";
    final double cIntakeIn = -1;
    final double cIntakeOut = 1;
    private HydraOpMode mOp;

    public HydraIntake(HydraOpMode op) {
        mOp = op;
        // initialize the motor object
        mMotPxlIntk = mOp.mHardwareMap.get(DcMotor.class, cfgIntakeMotor);
    }

    /**
     * Start running the intake to bring pixels in
     */
    public void StartIn() {
        mMotPxlIntk.setPower(cIntakeIn);
    }

    /**
     * Start running the intake to push pixels out
     */
    public void StartOut() {
        mMotPxlIntk.setPower(cIntakeOut);
    }

    /**
     * Stop running the intake
     */
    public void Stop() {
        mMotPxlIntk.setPower(0);
    }
}
