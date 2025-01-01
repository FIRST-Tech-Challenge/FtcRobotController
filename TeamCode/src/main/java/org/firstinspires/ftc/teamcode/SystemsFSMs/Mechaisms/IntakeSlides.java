package org.firstinspires.ftc.teamcode.SystemsFSMs.Mechaisms;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Hardware.Constants.IntakeConstants;
import org.firstinspires.ftc.teamcode.Hardware.Hardware;
import org.firstinspires.ftc.teamcode.Hardware.Util.Logger;

public class IntakeSlides {
    private DcMotorEx motor;
    private Logger logger;

    private double spoolDiam = 3.0; // Spool Diameter in cm
    private double extensionLimit = IntakeConstants.maxExtensionPosition; // Extension Limit in cm

    private final double ticksToCm = ( ( 24.0 / 17.0 ) * Math.PI * spoolDiam) / (145.1); // Multiply ticks by this number to get distance in cm
    private final double cmToTicks = 1 / ticksToCm; // Multiply cm by this number to get distance in encoder ticks

    private double currentTicks = 0;
    private double currentCM = 0;
    private double targetCM = 0;
    private double rangedTarget = 0;
    private double power = 0;
    private double current = 0;
    private double velocity = 0;

    private boolean encoderReset = false;

    private double
            p = IntakeConstants.sp,
            i = IntakeConstants.si,
            d = IntakeConstants.sd;

    private PIDController controller = new PIDController(p, i, d);

    public IntakeSlides(Hardware hardware, Logger logger) {
        motor = hardware.intakeSlideMotor;
        this.logger = logger;
    }

    public void update() {
        currentTicks = motor.getCurrentPosition();
        currentCM = currentTicks * ticksToCm;

        velocity = motor.getVelocity(AngleUnit.DEGREES);

        current = motor.getCurrent(CurrentUnit.MILLIAMPS);
    }

    public void command() {
        controller.setPID(p, i, d);

        rangedTarget = Math.min(Math.max(0, targetCM), extensionLimit);
        power = controller.calculate(currentCM * cmToTicks, rangedTarget * cmToTicks);

        // Re-Zero slides whenever target pos is zero
        if (targetCM == 0) {

            if (!encoderReset) {
                power = IntakeConstants.intakeSlideZeroPower;

                // Once the motor stalls, reset the encoder and set encoderReset to true
                if (current >= 7000 && velocity == 0) {
                    motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                    encoderReset = true;
                }

            } else {
                power = Math.min(power, IntakeConstants.intakeSlideZeroStallPower);
            }

        }

//        // If not at zero and target is zero, apply at least the slide zeroing power
//        if (targetCM == 0 && currentCM >= 0.00) {
//            power = Math.min(IntakeConstants.intakeSlideZeroPower, power);
//        } else if (targetCM == 0) {
//            power = IntakeConstants.intakeSlideZeroStallPower;
//        }


        if (targetCM != 0) {
            encoderReset = false;
        }


        motor.setPower(power);
    }

    public void log() {
        logger.log("<b>" + "Intake Slides" + "</b>", "", Logger.LogLevels.production);

        logger.log("Current CM", currentCM, Logger.LogLevels.debug);
        logger.log("Target CM", targetCM, Logger.LogLevels.debug);

        logger.log("Ranged Target CM", rangedTarget, Logger.LogLevels.developer);
        logger.log("Power", power, Logger.LogLevels.developer);
        logger.log("Intake Slide Velocity", velocity, Logger.LogLevels.developer);
        logger.log("Current", current, Logger.LogLevels.developer);
        logger.log("p", p, Logger.LogLevels.developer);
        logger.log("i", i, Logger.LogLevels.developer);
        logger.log("d", d, Logger.LogLevels.developer);
    }

    public void setTargetCM(double target) {
        targetCM = target;
    }

    public void setPID(double p, double i, double d) {
        this.p = p;
        this.i = i;
        this.d = d;
    }

    public double getPosition() {
        return currentCM;
    }

    public double getTargetCM() {
        return targetCM;
    }

    public double getVelocity() { return velocity; }

}