package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

public class Shooter {
    private static final String SHOOTER_MOTOR_NAME = "ShooterMotor"; // port 3
    private static final String TURRET_SERVO_NAME = "TurretServo";
    private static final double TICKS_PER_REV = 28.0; // GoBILDA 5203 6000 RPM
    private static final double BASE_RPM = 3000.0;
    private static final double RPM_STEP = 100.0;
    private static final double AT_SPEED_FRACTION = 0.98;
    private static final double FULL_SPEED_DELAY_SEC = 2.0;
    private static final double TURRET_SERVO_POWER = 0.5;
    private static final double TURRET_FULL_TRAVEL_SEC = 2.0;
    private static final double TRIGGER_DEADBAND = 0.05;
    private static final double RPM_SMOOTHING = 0.50;

    // 4500 RPM is the true physical max under load (2100 ticks/sec). Perfect F = 15.6.
    // Spin-up profile (I=0.0) prevents massive windup overshoot during acceleration.
    public static PIDFCoefficients SHOOTER_PIDF_SPINUP = new PIDFCoefficients(5.0, 0.0, 0, 15.6);
    // Steady-state profile (I=3.0) engages only when close to target to close the final gap.
    public static PIDFCoefficients SHOOTER_PIDF_STEADY = new PIDFCoefficients(5.0, 3.0, 0, 15.6);

    public enum State {
        IDLE, REVVING, AT_FULL_SPEED
    }

    private final DcMotorEx shooterMotor;
    private final CRServo turretServo;

    private double turretPower = 0.0;
    private double turretPosition = 0.0;
    private double targetRPM = BASE_RPM;
    private State state = State.IDLE;
    private boolean fireLatch = false;
    private boolean iZoneEngaged = false;

    private final ElapsedTime stateTimer = new ElapsedTime();

    private boolean prevDpadUp = false;
    private boolean prevDpadDown = false;
    private boolean prevX = false;

    private long lastTurretUpdateMs;
    private double currentRPM = 0;

    public Shooter(HardwareMap hardwareMap) {
        shooterMotor = hardwareMap.get(DcMotorEx.class, SHOOTER_MOTOR_NAME);
        turretServo = hardwareMap.get(CRServo.class, TURRET_SERVO_NAME);

        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, SHOOTER_PIDF_SPINUP);

        turretServo.setPower(0);
        lastTurretUpdateMs = System.currentTimeMillis();
    }

    public void update(Gamepad gamepad) {
        updateRPM();

        // --- Target RPM adjustment (edge-triggered, dpad up/down; x resets) ---
        boolean dpadUp = gamepad.dpad_up;
        boolean dpadDown = gamepad.dpad_down;
        boolean xBtn = gamepad.x;

        if (dpadUp && !prevDpadUp)
            targetRPM += RPM_STEP;
        if (dpadDown && !prevDpadDown)
            targetRPM = Math.max(0, targetRPM - RPM_STEP);
        if (xBtn && !prevX)
            targetRPM = BASE_RPM;

        prevDpadUp = dpadUp;
        prevDpadDown = dpadDown;
        prevX = xBtn;

        updateTurretPositionEstimate();

        // --- Turret rotation (hold dpad left/right) ---
        if (gamepad.dpad_left && !gamepad.dpad_right && turretPosition > 0.0) {
            turretPower = -TURRET_SERVO_POWER;
        } else if (gamepad.dpad_right && !gamepad.dpad_left && turretPosition < 1.0) {
            turretPower = TURRET_SERVO_POWER;
        } else {
            turretPower = 0;
        }
        turretServo.setPower(turretPower);

        // --- Dynamic iZone Management ---
        // If we are far from the target, disable the I-term to prevent massive windup.
        // If we are close (within 10%), enable the I-term to close the steady-state gap.
        if (state != State.IDLE && targetRPM > 0) {
            double errorRPM = Math.abs(currentRPM - targetRPM);
            if (!iZoneEngaged && errorRPM <= targetRPM * 0.10) {
                shooterMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, SHOOTER_PIDF_STEADY);
                iZoneEngaged = true;
            } else if (iZoneEngaged && errorRPM > targetRPM * 0.15) {
                shooterMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, SHOOTER_PIDF_SPINUP);
                iZoneEngaged = false;
            }
        }

        // --- Shooter state machine (right trigger) ---
        double trigger = gamepad.right_trigger;
        double targetVelocityTicks = (targetRPM / 60.0) * TICKS_PER_REV;

        switch (state) {
            case IDLE:
                if (trigger > TRIGGER_DEADBAND) {
                    // Reset iZone state on fresh spin-up
                    iZoneEngaged = false;
                    shooterMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, SHOOTER_PIDF_SPINUP);
                    
                    shooterMotor.setVelocity(targetVelocityTicks);
                    state = State.REVVING;
                }
                break;

            case REVVING:
                if (trigger <= TRIGGER_DEADBAND) {
                    stop();
                    state = State.IDLE;
                    break;
                }
                shooterMotor.setVelocity(targetVelocityTicks);

                // Only transition to AT_FULL_SPEED if we are within a +/- 5% tolerance of
                // target
                // This prevents firing when we overshoot the target during spin-up.
                double revvingTolerance = targetRPM * (1.0 - AT_SPEED_FRACTION);
                if (targetRPM > 0 && Math.abs(currentRPM - targetRPM) <= revvingTolerance) {
                    stateTimer.reset();
                    state = State.AT_FULL_SPEED;
                }
                break;

            case AT_FULL_SPEED:
                if (trigger <= TRIGGER_DEADBAND) {
                    stop();
                    fireLatch = false;
                    state = State.IDLE;
                    break;
                }
                shooterMotor.setVelocity(targetVelocityTicks);

                // Drop back to REVVING if we fall outside a slightly wider tolerance (e.g. 10%)
                double fullSpeedTolerance = targetRPM * (1.0 - AT_SPEED_FRACTION) * 2.0;
                if (targetRPM > 0 && Math.abs(currentRPM - targetRPM) > fullSpeedTolerance) {
                    state = State.REVVING;
                    break;
                }

                // Once we are stable in AT_FULL_SPEED, wait the delay before allowing a fire
                if (!fireLatch && stateTimer.seconds() >= FULL_SPEED_DELAY_SEC) {
                    fireLatch = true;
                }
                break;
        }
    }

    // Returns true exactly once per fire event and resets the 2-second countdown.
    public boolean consumeFire() {
        if (fireLatch) {
            fireLatch = false;
            stateTimer.reset();
            return true;
        }
        return false;
    }

    public double getRPM() {
        return currentRPM;
    }

    public double getTargetRPM() {
        return targetRPM;
    }

    public State getState() {
        return state;
    }

    public double getTurretPower() {
        return turretPower;
    }

    public double getTurretPosition() {
        return turretPosition;
    }

    public void stop() {
        shooterMotor.setVelocity(0);
        turretServo.setPower(0);
        turretPower = 0;
    }

    private void updateRPM() {
        double rawRPM = Math.abs(shooterMotor.getVelocity()) / TICKS_PER_REV * 60.0;
        currentRPM += (rawRPM - currentRPM) * RPM_SMOOTHING;
    }

    private void updateTurretPositionEstimate() {
        long currentTimeMs = System.currentTimeMillis();
        double deltaSeconds = (currentTimeMs - lastTurretUpdateMs) / 1000.0;
        lastTurretUpdateMs = currentTimeMs;

        turretPosition += Math.signum(turretPower) * deltaSeconds / TURRET_FULL_TRAVEL_SEC;
        turretPosition = Math.max(0.0, Math.min(1.0, turretPosition));
    }
}
