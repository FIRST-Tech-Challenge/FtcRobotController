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
    // Feedforward adds VELOCITY_F * target ticks/s to the PID power. Start near
    // 1 / (full-power ticks/s), e.g. 1 / 2800 ≈ 0.00036, then lower I.
    public static boolean USE_F = false;
    public static double VELOCITY_F = 0;
    // Flag the encoders when they disagree by more than this fraction of the
    // faster reading; below the minimum speed the ratio is too noisy to judge.
    public static double ENCODER_MISMATCH_RATIO = 0.2;
    public static double ENCODER_CHECK_MIN_VEL = 300;
    private final DcMotorEx shooterLeft, shooterRight;
    private final ElapsedTime loopTimer = new ElapsedTime();
    private double commandedVel;
    private double integral;
    private double previousError;
    private boolean hasPreviousError;
    private boolean usingPid;
    private boolean usingF;
    private boolean encoderMismatch;

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
        usingF = USE_F;
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
        double measuredVel = measureVelocity();
        if (!usingPid) {
            shooterLeft.setVelocity(commandedVel);
            shooterRight.setVelocity(commandedVel);
            return;
        }
        if (usingF != USE_F) {
            // The integral was built for the old feedforward; start it fresh.
            usingF = USE_F;
            resetController();
        }

        double dt = Math.max(loopTimer.seconds(), 0.001);
        loopTimer.reset();
        double error = commandedVel - measuredVel;
        double slope = hasPreviousError ? (error - previousError) / dt : 0.0;
        double feedforward = usingF ? VELOCITY_F * commandedVel : 0.0;
        double base = feedforward + VELOCITY_P * error + VELOCITY_D * slope;
        double candidateIntegral = integral + error * dt;
        double trial = base + VELOCITY_I * candidateIntegral;

        // Integrate only when unsaturated or when the error unwinds saturation.
        if ((trial >= -1.0 && trial <= 1.0)
                || (trial > 1.0 && error < 0.0)
                || (trial < -1.0 && error > 0.0)) {
            integral = candidateIntegral;
        }
        if (VELOCITY_I == 0.0) {
            // Drop any integral left over from before I was set to zero.
            integral = 0.0;
        }
        double output = base + VELOCITY_I * integral;
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
        encoderMismatch = false;
        resetController();
    }

    /**
     * Both motors spin one flywheel, so their encoders should agree. When they
     * do not, an unplugged or dead encoder reads near zero, so trust the faster one.
     */
    private double measureVelocity() {
        double left = shooterLeft.getVelocity();
        double right = shooterRight.getVelocity();
        double faster = Math.max(Math.abs(left), Math.abs(right));
        encoderMismatch = faster >= ENCODER_CHECK_MIN_VEL
                && Math.abs(left - right) > ENCODER_MISMATCH_RATIO * faster;
        if (!encoderMismatch) {
            return (left + right) / 2;
        }
        return Math.abs(left) >= Math.abs(right) ? left : right;
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

    /** True while the two shooter encoders disagree; check the wiring. */
    public boolean isEncoderMismatch() {
        return encoderMismatch;
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
