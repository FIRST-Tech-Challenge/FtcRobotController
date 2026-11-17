
package org.firstinspires.ftc.teamcode.Core;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class MotorWrapper {
    private final MotorEx motorEx;
    private double lastReadVelocity;
    private double lastReadPosition;
    private double ratio;
    private final double TICK_PER_REVOLUTION;

    public MotorWrapper(MotorEx motorEx, boolean velocityControl, double ratio, boolean inverse) {
        this(motorEx, velocityControl, ratio, inverse, (motorEx != null && motorEx.getCPR() > 0) ? motorEx.getCPR() : 145.1);
    }

    public MotorWrapper(MotorEx motorEx, boolean velocityControl, double ratio, boolean inverse, double ticksPerRev) {
        this.motorEx = motorEx;
        this.TICK_PER_REVOLUTION = ticksPerRev > 0 ? ticksPerRev : 145.1;
        if (velocityControl && motorEx != null) {
            motorEx.setRunMode(Motor.RunMode.VelocityControl);

            if (inverse) {
                motorEx.motor.setDirection(DcMotorSimple.Direction.REVERSE);
            }
        }
        this.ratio = ratio;
    }


    // POWER

    /**
     * Description: The set method is a wrapper of the motor set method
     *
     * @param : the power to set the motor at
     */

    public void set(double power) {
        motorEx.set(power);
    }

    public double get() {
        return motorEx.get();
    }

    // VELOCITY

    /**
     * Description: The following method reads the encoder to get velocity of the motor
     *
     * @return: the velocity that is just read
     */

    public void readVelocity() {
        lastReadVelocity = motorEx.getCorrectedVelocity();
    }


    /**
     * Description: The following method gets the velocity of the motor that was last read
     *
     * @return: the velocity that was read before
     */

    public double getVelocity() {
        return lastReadVelocity;
    }

    public double getRPM() {
        return (lastReadVelocity * 60.0) / TICK_PER_REVOLUTION;
    }

    public void setVelocity(double targetVelocity) {
        motorEx.setVelocity(targetVelocity);
    }

    public void setRPM(double targetRPM) {
        double targetTicksPerSec = (targetRPM * TICK_PER_REVOLUTION) / 60.0;
        motorEx.setVelocity(targetTicksPerSec);
    }

    public void setVelocityConstants(double vF, double vP, double vI, double vD) {
        motorEx.setVeloCoefficients(vP, vI, vD);
        motorEx.setFeedforwardCoefficients(0, vF);
    }

    public void setVelocityConstants(double p, double i, double d, double kS, double kV, double kA) {
        motorEx.setVeloCoefficients(p, i, d);
        motorEx.setFeedforwardCoefficients(kS, kV, kA);
    }

    // POSITION

    public void readPosition() {
        lastReadPosition = (motorEx.getCurrentPosition() / TICK_PER_REVOLUTION) * 360;
    }

    public double getTicksPerRev() {
        return TICK_PER_REVOLUTION;
    }

    public double readandGetPosTicks() {
        return motorEx.getCurrentPosition();
    }

    public double getAngle() {
        return lastReadPosition;
    }

    public double getScaledPos() {
        return lastReadPosition * ratio;
    }

    public double getCurrent() {
        return motorEx.motorEx.getCurrent(CurrentUnit.AMPS);
    }

    public void resetEncoder() {
        motorEx.stopAndResetEncoder();
    }

}