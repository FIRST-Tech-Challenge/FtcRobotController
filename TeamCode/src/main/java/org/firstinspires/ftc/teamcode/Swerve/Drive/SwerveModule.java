package org.firstinspires.ftc.teamcode.Swerve.Drive;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeRadians;
import static java.lang.Math.cos;
import static java.lang.Math.signum;

import androidx.core.math.MathUtils;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Core.Logger;

@Config
public class SwerveModule {

    private double MAX_SERVO = 1, MAX_MOTOR = 1;

    private DcMotorEx motor;
    private CRServoImplEx servo;
    private AnalogInput encoder;
    private PIDController rotationController;

    public boolean wheelFlipped = false;
    private double target = 0.0;
    private double current = 0.0;
    private boolean inveresed = false;
    double lastMotorPower;
    double lastServoPower;
    double lastTargetPosition;

    private double k_static = 0.00, p = 0.5, i = 0.0, d = 0.008;
    private double offset;

    Logger logger;

    public SwerveModule(DcMotorEx m, CRServoImplEx s, AnalogInput e, Double o, Boolean inv, Logger logger) {
        motor = m;
        MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
        motorConfigurationType.setAchieveableMaxRPMFraction(MAX_MOTOR);
        motor.setMotorType(motorConfigurationType);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        servo = s;
        servo.setPwmRange(new PwmControl.PwmRange(500, 2500, 5000));

        encoder = e;
        offset = o;
        inveresed = inv;

        rotationController = new PIDController(p, i, d);

        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Initialize target to current encoder position to prevent startup movement
        initializePosition();

        this.logger = logger;
    }

    /**
     * Call this once at startup to set the target angle to the current encoder position,
     * preventing the steering servo from spinning on the first update().
     */
    public void initializePosition() {
        target = normalizeRadians(getCurrentRotation());
        rotationController.reset();
        lastTargetPosition = target < 0 ? target + 2 * Math.PI : target;
        servo.setPower(0);
        motor.setPower(0);
    }

    public void update(double wa, double ws) {
        if (Double.isNaN(wa) || Double.isNaN(ws)) {
            rotationController.reset();
            wa = 0;
            ws = 0;
        }

        rotationController.setPID(p, i, d);

        double current = normalizeRadians(getCurrentRotation());
        double target = normalizeRadians(wa);

        double error = normalizeRadians(target - current);

        if (Math.abs(error) > Math.PI / 2) {
            target = normalizeRadians(target + Math.PI);
            error = normalizeRadians(target - current);
            ws *= -1;
            wheelFlipped = true;
        } else {
            wheelFlipped = false;
        }

        lastTargetPosition = target < 0 ? target + 2 * Math.PI : target;

        if (Math.abs(error) < 0.02) {
            error = 0;
        }

        double power = Range.clip(rotationController.calculate(0, error), -MAX_SERVO, MAX_SERVO);
        if (Double.isNaN(power)) power = 0;

        servo.setPower(power + (Math.abs(error) > 0.02 ? signum(power) * k_static : 0));
        motor.setPower((ws) * cos(error));
        lastMotorPower = ws;
        lastServoPower = power;
    }

    public double getCurrentRotation() {
        double encoderVoltage = MathUtils.clamp(encoder.getVoltage(), 0, 3.3);
        double pos;

        if (!inveresed) {
            pos = (encoderVoltage / 3.3) * 2 * Math.PI - offset;
        } else {
            pos = (1 - encoderVoltage / 3.3) * 2 * Math.PI - offset;
        }
        return pos;
    }

    public double getTargetRotation() {
        return target;
    }

    public void setTargetRotation(double target) {
        this.target = target;
    }

    public void setOffset(double o) {
        offset = o;
    }

    public double getOffset() {
        return offset;
    }

    public void setInverse(boolean inveresed) {
        this.inveresed = inveresed;
    }

    public boolean getInverse() {
        return inveresed;
    }

    public void setMotorPower(double power) {
        motor.setPower(power);
    }

    public double getMotorPower() {
        return lastMotorPower;
    }

    public void setPID(double p, double i, double d) {
        this.p = p;
        this.i = i;
        this.d = d;
    }

    public double getP() {
        return p;
    }

    public double getI() {
        return i;
    }

    public double getD() {
        return d;
    }

    public void setMode(DcMotor.RunMode runMode) {
        motor.setMode(runMode);
    }

    public double getMotorCurrent() {
        return motor.getCurrent(CurrentUnit.AMPS);
    }

    public void log() {
        logger.log("Motor power", lastMotorPower, Logger.LogLevels.DEBUG);
        logger.log("Wheel Flipped", wheelFlipped, Logger.LogLevels.DEBUG);
        logger.log("Target Position", lastTargetPosition, Logger.LogLevels.DEBUG);
        logger.log("Current Position", getCurrentRotation(), Logger.LogLevels.DEBUG);
        logger.log("Servo Power", lastServoPower, Logger.LogLevels.DEBUG);
        logger.log("Motor Current", motor.getCurrent(CurrentUnit.AMPS), Logger.LogLevels.DEBUG);
    }
}