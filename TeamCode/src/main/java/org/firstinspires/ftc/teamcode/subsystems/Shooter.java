package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.jetbrains.annotations.NotNull;

/** Owns the shooter motor; the TeleOp decides when to fire. */
@Configurable
public final class Shooter extends SubsystemBase {
    // Shared velocity in encoder ticks per second; tune in Panels.
    public static double SHOOT_VEL = 1560;
    // Opt in after tuning; false uses the Hub's existing velocity controller.
    public static boolean USE_PID = true;
    // Custom PID gains produce motor power from ticks/s error. Tune in Panels;
    // Hub PIDF tuner coefficients are incompatible. Zeros leave the motor off.
    public static double VELOCITY_P = 0.01;
    public static double VELOCITY_I = 0.008;
    public static double VELOCITY_D = 0;
    private final DcMotorEx shooterLeft, shooterRight;
    private final ElapsedTime loopTimer = new ElapsedTime();
    private double commandedVel;
    private double integral;
    private double previousError;
    private boolean hasPreviousError;
    private boolean usingPid;

    public Shooter(@NotNull final HardwareMap hMap) {
        shooterLeft = hMap.get(DcMotorEx.class, "shooterLeft");
        shooterLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        shooterLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooterLeft.setPower(0.0);

        shooterRight = hMap.get(DcMotorEx.class, "shooterRight");
        shooterRight.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooterRight.setPower(0.0);

        usingPid = USE_PID;
        shooterLeft.setMode(usingPid ? DcMotor.RunMode.RUN_WITHOUT_ENCODER
                : DcMotor.RunMode.RUN_USING_ENCODER);
        shooterRight.setMode(usingPid ? DcMotor.RunMode.RUN_WITHOUT_ENCODER
                : DcMotor.RunMode.RUN_USING_ENCODER);
        stop();
    }

    public void shoot() {
        setVelocity(SHOOT_VEL);
    }

    public void setVelocity(double velocity) {
        updateControlMode();
        if (velocity == 0.0) {
            stop();
            return;
        }
        if (velocity != commandedVel) {
            resetController();
        }
        commandedVel = velocity;
        if (!usingPid) {
            shooterLeft.setVelocity(velocity);
            shooterRight.setVelocity(velocity);
        }
    }

    @Override
    public void periodic() {
        updateControlMode();
        if (commandedVel == 0.0) {
            return;
        }
        if (!usingPid) {
            shooterLeft.setVelocity(commandedVel);
            shooterRight.setVelocity(commandedVel);
            return;
        }

        double dt = Math.max(loopTimer.seconds(), 0.001);
        loopTimer.reset();
        double error = commandedVel - (shooterLeft.getVelocity() + shooterRight.getVelocity()) / 2;
        double slope = hasPreviousError ? (error - previousError) / dt : 0.0;
        double candidateIntegral = VELOCITY_I == 0.0 ? 0.0 : integral + error * dt;
        double output = VELOCITY_P * error + VELOCITY_I * candidateIntegral
                + VELOCITY_D * slope;

        // Integrate only when unsaturated or when the error unwinds saturation.
        if ((output >= -1.0 && output <= 1.0)
                || (output > 1.0 && error < 0.0)
                || (output < -1.0 && error > 0.0)) {
            integral = candidateIntegral;
        }
        if (VELOCITY_I == 0.0) {
            integral = 0.0;
        }
        output = VELOCITY_P * error + VELOCITY_I * integral + VELOCITY_D * slope;
        previousError = error;
        hasPreviousError = true;

        shooterLeft.setPower(Math.max(-1.0, Math.min(1.0, output)));
        shooterRight.setPower(Math.max(-1.0, Math.min(1.0, output)));
    }

    public void stop() {
        if (usingPid) {
            shooterLeft.setPower(0.0);
            shooterRight.setPower(0.0);
        } else {
            // Use velocity consistently so SDK power caching cannot skip the stop.
            shooterLeft.setVelocity(0.0);
            shooterRight.setVelocity(0.0);
        }
        commandedVel = 0.0;
        resetController();
    }

    private void updateControlMode() {
        if (usingPid == USE_PID) {
            return;
        }
        // Stop in the old mode before switching, including live Panels changes.
        if (usingPid) {
            shooterLeft.setPower(0.0);
            shooterRight.setPower(0.0);
        } else {
            shooterLeft.setVelocity(0.0);
            shooterRight.setVelocity(0.0);
        }
        usingPid = USE_PID;
        shooterLeft.setMode(usingPid ? DcMotor.RunMode.RUN_WITHOUT_ENCODER
                : DcMotor.RunMode.RUN_USING_ENCODER);
        shooterRight.setMode(usingPid ? DcMotor.RunMode.RUN_WITHOUT_ENCODER
                : DcMotor.RunMode.RUN_USING_ENCODER);
        resetController();
    }

    private void resetController() {
        integral = 0.0;
        previousError = 0.0;
        hasPreviousError = false;
        loopTimer.reset();
    }

    /** Requested velocity in ticks/s, not an RPM or motion measurement. */
    public double getCommandedVel() {
        return commandedVel;
    }

    public double getLeftVel() {
        return shooterLeft.getVelocity();
    }

    public double getRightVel() {
        return shooterRight.getVelocity();
    }

    public double getLeftPow() {
        return shooterLeft.getPower();
    }

    public double getRightPow() {
        return shooterRight.getPower();
    }
}
