package org.firstinspires.ftc.teamcode.Systems.Mechaisms;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Hardware.Constants.DepositConstants;
import org.firstinspires.ftc.teamcode.Hardware.Hardware;
import org.firstinspires.ftc.teamcode.Systems.Deposit;

public class DepositSlides {
    private DcMotorEx rightMotor;
    private DcMotorEx leftMotor;

    private double spoolDiam = 3.0; // Spool Diameter in cm
    private double extensionLimit = 72; // Extension Limit in cm

    private final double ticksToCm = ((1.0/4.0) * Math.PI * spoolDiam) / (28); // Multiply ticks by this number to get distance in cm
    private final double cmToTicks = 1 / ticksToCm; // Multiply cm by this number to get distance in encoder ticks

    private double currentCM = 0;
    private double targetCM = 0;
    private double rangedTarget = 0;
    private double power = 0;

    private double f = DepositConstants.sf;



    private PIDController controller = new PIDController(DepositConstants.sp, DepositConstants.si, DepositConstants.sd);

    public DepositSlides(Hardware hardware) {
        rightMotor = hardware.depositSlideRight;
        leftMotor = hardware.depositSlideLeft;
    }

    public void update() {
        currentCM = rightMotor.getCurrentPosition() * ticksToCm;
    }

    public void command() {
        rangedTarget = Math.min(Math.max(0, targetCM), extensionLimit);
        power = controller.calculate(currentCM * cmToTicks, rangedTarget * cmToTicks) + f;
        rightMotor.setPower(power);
        leftMotor.setPower(power);
    }

    public void setTargetCM(double target) {
        targetCM = target;
    }

    public void setPID(double p, double i, double d, double f) {
        controller.setPID(p, i, d);
        this.f = f;
    }

    public double getPosition() {
        return currentCM;
    }

    public double getTarget() {
        return targetCM;
    }

    public double getCurrent() {
        return rightMotor.getCurrent(CurrentUnit.MILLIAMPS) + leftMotor.getCurrent(CurrentUnit.MILLIAMPS);
    }

    public double getPower() {
        return power;
    }

    public double getRangedTarget() {
        return rangedTarget;
    }

}
