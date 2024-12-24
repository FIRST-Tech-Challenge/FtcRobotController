package org.firstinspires.ftc.teamcode.SystemsFSMs.Mechaisms;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Hardware.Constants.DepositConstants;
import org.firstinspires.ftc.teamcode.Hardware.Hardware;
import org.firstinspires.ftc.teamcode.Hardware.Util.Logger;

public class DepositSlides {
    private DcMotorEx rightMotor;
    private DcMotorEx leftMotor;
    private Logger logger;

    private double spoolDiam = 3.0; // Spool Diameter in cm
    private double extensionLimit = DepositConstants.slideMaxExtension; // Extension Limit in cm

    private final double ticksToCm = (( 12.0 / 48.0 ) * Math.PI * spoolDiam) / (28); // Multiply ticks by this number to get distance in cm
    private final double cmToTicks = 1 / ticksToCm; // Multiply cm by this number to get distance in encoder ticks

    private double currentTicks = 0;
    private double currentCM = 0;
    private double targetCM = 0;
    private double rangedTarget = 0;
    private double power = 0;
    private double rightCurrent = 0;
    private double leftCurrent = 0;
    private double totalCurrent = 0;

    private double
            p = DepositConstants.sp,
            i = DepositConstants.si,
            d = DepositConstants.sd,
            f = DepositConstants.sf;



    private PIDController controller = new PIDController(DepositConstants.sp, DepositConstants.si, DepositConstants.sd);

    public DepositSlides(Hardware hardware, Logger logger) {
        rightMotor = hardware.depositSlideRight;
        leftMotor = hardware.depositSlideLeft;

        this.logger = logger;
    }

    public void update() {
        currentTicks = rightMotor.getCurrentPosition();
        currentCM = -currentTicks * ticksToCm;

        rightCurrent = rightMotor.getCurrent(CurrentUnit.MILLIAMPS);
        leftCurrent = leftMotor.getCurrent(CurrentUnit.MILLIAMPS);
        totalCurrent = rightCurrent + leftCurrent;


    }

    public void command() {
        controller.setPID(p, i, d);

        rangedTarget = Math.min(Math.max(0, targetCM), extensionLimit);
        power = controller.calculate(currentCM * cmToTicks, rangedTarget * cmToTicks) + f;
        rightMotor.setPower(power);
        leftMotor.setPower(power);
    }

    public void log() {
        logger.log("Deposit Slides", "", Logger.LogLevels.production);

        logger.log("Current CM", currentCM, Logger.LogLevels.debug);
        logger.log("Target CM", targetCM, Logger.LogLevels.debug);

        logger.log("Ranged Target CM", rangedTarget, Logger.LogLevels.developer);
        logger.log("Power", power, Logger.LogLevels.developer);
        logger.log("Right Current", rightCurrent, Logger.LogLevels.developer);
        logger.log("Left Current", leftCurrent, Logger.LogLevels.developer);
        logger.log("Total Current", totalCurrent, Logger.LogLevels.developer);
        logger.log("p", p, Logger.LogLevels.developer);
        logger.log("i", i, Logger.LogLevels.developer);
        logger.log("d", d, Logger.LogLevels.developer);
        logger.log("f", f, Logger.LogLevels.developer);
    }

    public void setTargetCM(double target) {
        targetCM = target;
    }

    public void setPID(double p, double i, double d, double f) {
        this.p = p;
        this.i = i;
        this.d = d;
        this.f = f;
    }

    public double getPosition() {
        return currentCM;
    }

    public double getTargetCM() {
        return targetCM;
    }

}
