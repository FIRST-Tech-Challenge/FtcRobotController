package org.firstinspires.ftc.teamcode.rework.Modules;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ReworkDrivetrain extends Module {
    private DcMotor fLeft;
    private DcMotor fRight;
    private DcMotor bLeft;
    private DcMotor bRight;

    private final static double POWER_SCALE_FACTOR = 0.9;

    public ReworkDrivetrain(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        init();
    }

    protected void init() {
        fLeft = getDcMotor("fLeft");
        fRight = getDcMotor("fRight");
        bLeft = getDcMotor("bLeft");
        bRight = getDcMotor("bRight");

        setDrivetrainZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void drive(double forward, double rotate, double mecanum) {
        double fLPower = (-forward + rotate + mecanum) * POWER_SCALE_FACTOR;
        double fRPower = (-forward - rotate - mecanum) * POWER_SCALE_FACTOR;
        double bLPower = (-forward + rotate - mecanum) * POWER_SCALE_FACTOR;
        double bRPower = (-forward - rotate + mecanum) * POWER_SCALE_FACTOR;

        drive(fLPower, fRPower, bLPower, bRPower);
    }

    public void drive(double fLPower, double fRPower, double bLPower, double bRPower) {
        fLeft.setPower(fLPower);
        fRight.setPower(fRPower);
        bLeft.setPower(bLPower);
        bRight.setPower(bRPower);
    }

    private void setDrivetrainZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        fLeft.setZeroPowerBehavior(zeroPowerBehavior);
        fRight.setZeroPowerBehavior(zeroPowerBehavior);
        bLeft.setZeroPowerBehavior(zeroPowerBehavior);
        bRight.setZeroPowerBehavior(zeroPowerBehavior);
    }
}
