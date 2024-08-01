package org.firstinspires.ftc.teamcode.NewStuff;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class DriveTrain {
    private OpModeUtilities opModeUtilities;

    private DcMotor fLeft;
    private DcMotor fRight;
    private DcMotor bLeft;
    private DcMotor bRight;

    public DriveTrain(OpModeUtilities opModeUtilities) {
        this.opModeUtilities = opModeUtilities;
        setUpHardware();
    }

    private void setUpHardware() {
        fLeft = opModeUtilities.getHardwareMap().dcMotor.get("fLeft");
        fRight = opModeUtilities.getHardwareMap().dcMotor.get("fRight");
        bLeft = opModeUtilities.getHardwareMap().dcMotor.get("bLeft");
        bRight = opModeUtilities.getHardwareMap().dcMotor.get("bRight");

        fLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        fRight.setDirection(DcMotorSimple.Direction.FORWARD);
        bLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        bRight.setDirection(DcMotorSimple.Direction.FORWARD);

        fLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void setFLeftPower(double power) { fLeft.setPower(power); }
    public void setFRightPower(double power) { fRight.setPower(power); }
    public void setBLeftPower(double power) { bLeft.setPower(power); }
    public void setBRightPower(double power) { bRight.setPower(power); }

    public void setPower (double fLeftPower, double fRightPower, double bLeftPower, double bRightPower){
        setFLeftPower(fLeftPower);
        setFRightPower(fRightPower);
        setBLeftPower(bLeftPower);
        setBRightPower(bRightPower);
    }

    public DcMotor getfLeft() {
        return fLeft;
    }

    public DcMotor getfRight() {
        return fRight;
    }

    public DcMotor getbLeft() {
        return bLeft;
    }

    public DcMotor getbRight() {
        return bRight;
    }

}
