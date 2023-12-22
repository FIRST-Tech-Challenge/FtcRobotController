package org.firstinspires.ftc.team6220_CENTERSTAGE;

import org.firstinspires.ftc.team6220_CENTERSTAGE.Constants;
import org.firstinspires.ftc.team6220_CENTERSTAGE.MecanumDrive;

/*
This class adds features that are not built into the roadrunner mecanum drive class.
It enables us to control our different mechanisms like slides/intake/etc.
 */
public class ExtendedDriveFeatures {

    MecanumDrive drive;

    public ExtendedDriveFeatures(MecanumDrive drive) {
        this.drive=drive;
    }

    /**
     * moves the slides at a power
     * @param power positive is slides up
     */
    public void moveSlides(double power) {
        this.drive.slideMotor.setPower(power);
        // if the power is negative, which is slides going down
        if (power > 0) {
            this.drive.returnMotor.setPower(power * Constants.SLIDE_RETURN_UP_MUL);
        } else {
            this.drive.returnMotor.setPower(power * Constants.SLIDE_RETURN_POWER_MULTIPLIER + Constants.SLIDE_RETURN_POWER_OFFSET);
        }
    }

    /**
     * move slides until within a position tolerance
     * @param targetPos the destination position of the slides
     * @return true if it is NOT yet close enough to target pos
     */
    public boolean moveSlidesPreset(int targetPos) {
        double power = (targetPos - this.drive.slideMotor.getCurrentPosition()) * Constants.SLIDE_P_GAIN;
        this.moveSlides(clamp(power, -Constants.AUTO_SLIDES_MAX_SPEED, Constants.AUTO_SLIDES_MAX_SPEED));
        return Math.abs(power) >= Constants.AUTO_SLIDES_PRESET_TOLERANCE;
    }

    // PLACEHOLDER UNTIL WE HAVE THE ARM
    /* public boolean moveSuspensionArmPreset(int targetPos) {
        double power = (targetPos - this.drive.motSuspension.getCurrentPosition()) * Constants.SLIDE_P_GAIN;
        this.drive.motSuspension.setPower(clamp(power, -Constants.AUTO_SLIDES_MAX_SPEED, Constants.AUTO_SLIDES_MAX_SPEED));
        return Math.abs(power) >= Constants.AUTO_SLIDES_PRESET_TOLERANCE;
    } */

    // clamps value between a minimum and maximum value
    private static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

}
