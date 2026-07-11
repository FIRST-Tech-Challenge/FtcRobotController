package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Shooter {
    private static final String SHOOTER_MOTOR_NAME  = "ShooterMotor"; // port 3
    private static final String TURRET_SERVO_NAME   = "TurretServo";
    private static final double TICKS_PER_REV        = 28.0; // GoBILDA 5203 6000 RPM
    private static final double PEAK_RPM             = 3000.0;
    private static final double AT_SPEED_FRACTION    = 0.95;
    private static final double FULL_SPEED_DELAY_SEC = 2.0;
    private static final double TURRET_SERVO_POWER   = 0.5;
    private static final double TURRET_FULL_TRAVEL_SEC = 2.0;
    private static final double TRIGGER_DEADBAND     = 0.05;
    private static final double RPM_SMOOTHING        = 0.20;

    public enum State { IDLE, REVVING, AT_FULL_SPEED }

    private final DcMotorEx shooterMotor;
    private final CRServo   turretServo;

    private int     maxSpeedPercent = 100;
    private double  turretPower     = 0.0;
    private double  turretPosition  = 0.0;
    private double  targetRPM       = 0.0;
    private State   state           = State.IDLE;
    private boolean fireLatch       = false;

    private final ElapsedTime stateTimer = new ElapsedTime();

    private boolean prevDpadUp   = false;
    private boolean prevDpadDown = false;
    private boolean prevX        = false;

    private long lastTurretUpdateMs;
    private double currentRPM = 0;

    public Shooter(HardwareMap hardwareMap) {
        shooterMotor = hardwareMap.get(DcMotorEx.class, SHOOTER_MOTOR_NAME);
        turretServo  = hardwareMap.get(CRServo.class, TURRET_SERVO_NAME);

        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        turretServo.setPower(0);
        lastTurretUpdateMs = System.currentTimeMillis();
    }

    public void update(Gamepad gamepad) {
        updateRPM();

        // --- Max speed (edge-triggered, dpad up/down; x resets) ---
        boolean dpadUp   = gamepad.dpad_up;
        boolean dpadDown = gamepad.dpad_down;
        boolean xBtn     = gamepad.x;

        if (dpadUp && !prevDpadUp && maxSpeedPercent < 100)
            maxSpeedPercent = Math.min(100, maxSpeedPercent + 10);
        if (dpadDown && !prevDpadDown && maxSpeedPercent > 10)
            maxSpeedPercent = Math.max(10, maxSpeedPercent - 10);
        if (xBtn && !prevX)
            maxSpeedPercent = 100;

        prevDpadUp   = dpadUp;
        prevDpadDown = dpadDown;
        prevX        = xBtn;

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

        // --- Shooter state machine (right trigger) ---
        double trigger     = gamepad.right_trigger;
        double targetPower = trigger * (maxSpeedPercent / 100.0);
        targetRPM = PEAK_RPM * targetPower;

        switch (state) {
            case IDLE:
                if (trigger > TRIGGER_DEADBAND) {
                    shooterMotor.setPower(targetPower);
                    state = State.REVVING;
                }
                break;

            case REVVING:
                if (trigger <= TRIGGER_DEADBAND) {
                    stop();
                    state = State.IDLE;
                    break;
                }
                shooterMotor.setPower(targetPower);
                if (targetRPM > 0 && currentRPM >= targetRPM * AT_SPEED_FRACTION) {
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
                shooterMotor.setPower(targetPower);
                if (targetRPM > 0 && currentRPM < targetRPM * AT_SPEED_FRACTION * 0.9) {
                    state = State.REVVING;
                    break;
                }
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

    public double  getRPM()             { return currentRPM; }
    public double  getTargetRPM()       { return targetRPM; }
    public State   getState()           { return state; }
    public int     getMaxSpeedPercent() { return maxSpeedPercent; }
    public double  getTurretPower()     { return turretPower; }
    public double  getTurretPosition()  { return turretPosition; }

    public void stop() {
        shooterMotor.setPower(0);
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
