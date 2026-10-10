
package org.firstinspires.ftc.teamcode.Core;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class MotorWrapper {
    private final MotorEx motorEx;
    private double lastReadVelocity;
    private double lastReadPosition;
    private double ratio;
    private final double TICK_PER_REVOLUTION;

    public MotorWrapper(MotorEx motorEx, boolean velocityControl, double ratio, boolean inverse) {
        this.motorEx = motorEx;
        this.TICK_PER_REVOLUTION = 145.1;
        if (velocityControl) {
            motorEx.setRunMode(Motor.RunMode.VelocityControl);

            if (inverse) {
                motorEx.motor.setDirection(DcMotorSimple.Direction.REVERSE);
            }

        }
        this.ratio = ratio;
    }

    public void readVelocity() {
        lastReadVelocity = motorEx.getCorrectedVelocity();
    }

    public double getVelocity() {
        return lastReadVelocity;
    }

    public void setVelocity(double targetVelocity) {
        motorEx.setVelocity(targetVelocity);
    }

    public void setVelocityConstants(double vP, double vI, double vD, double ks, double kv, double ka) {
        motorEx.setVeloCoefficients(vP, vI, vD);
        motorEx.setFeedforwardCoefficients(ks, kv, ka);

    }

    public double getCurrent() {
        return motorEx.motorEx.getCurrent(CurrentUnit.AMPS);
    }
}