package org.firstinspires.ftc.teamcode.methods;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

public class Transfer {
    // Voltage-model gains validated by TransferAutoTuning and single-ball tests.
    // These are not REV Hub PIDFCoefficients. KV must be positive before use.
    public static double KS = 1.1294224, KV = 0.004082160;
    public static double KP = 0.001;
    public static double SHOOTING_VELOCITY = 1440;

    // Copy the confirmed settings from the tuner. With both motors configured
    // FORWARD as below, the POWER_SIGN values must match the tuning OpMode.
    // ENCODER_SIGN_0 must match its learned "Encoder signs M0" value.
    public static double MOTOR0_POWER_SIGN = 1.0;
    public static double MOTOR1_POWER_SIGN = 1.0;
    public static double ENCODER_SIGN_0 = 1.0;

    public static final double OUTTAKE_POWER = 0.40;
    private boolean outtaking;

    private boolean wasOn = false;

    // Transfer power is now a -1..1 duty-cycle setting, not ticks/second.
    public static double MECH_POWER = 1.0;
    // Starting examples: verify readiness settings on the robot.
    public static double VELOCITY_TOLERANCE = 40.0;
    public static double READY_HOLD_SECONDS = 0.15;
    /** Legacy setting retained for compatibility; readiness timeouts are disabled. */
    public static double FEED_READY_TIMEOUT_SECONDS = 3.0;

    private final BatteryVoltage battery;
    private final DcMotorEx fWheel;
    private boolean flywheelsOn = false;
    private boolean fireRequested = false, firing = false, ejecting = false;
    private boolean feedStarted = false;
    private double targetVelocity = 0.0, measuredVelocity = 0.0;
    private double batteryVoltage = 12.76, requestedVolts = 0.0, motorPower = 0.0;
    private long readySince = 0, fireRequestedAt = 0;
    private boolean ballJustShot = false;
    private boolean flywheelDropped = false;
    // only count drops once the flywheel has been up to speed (ignores spin-up)
    private boolean flywheelReady = false;
    private double flywheelThreshold = 1200;

    public Transfer(HardwareMap hardwareMap) {
        this(hardwareMap, new BatteryVoltage(hardwareMap));
    }

    /** Preferred: share one BatteryVoltage between Launcher, Transfer and Intake. */
    public Transfer(HardwareMap hardwareMap, BatteryVoltage battery) {
        this(hardwareMap.get(DcMotorEx.class, "LaunchMech"), battery);
    }

    Transfer(DcMotorEx motor, Iterable<VoltageSensor> sensors) {
        this(motor, new BatteryVoltage(sensors));
    }

    Transfer(DcMotorEx motor, BatteryVoltage battery) {
        this.battery = battery;
        fWheel = motor;
        configureFlywheel(fWheel);
        //stopAll();
    }

    private void configureFlywheel(DcMotorEx motor) {
        motor.setPower(0.0);
        motor.setDirection(DcMotorSimple.Direction.FORWARD);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // BRAKE so a paced stop halts the next ball quickly instead of coasting it
        // into the flywheel. Forward control never commands 0 while feeding.
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void toggleOn() {
        if (!wasOn) {
            setTargetVelocity(SHOOTING_VELOCITY);
            wasOn = true;
        }
        else {
            stopAll();
            wasOn = false;
        }
    }


    public void setTargetVelocity(double ticksPerSecond) {
        if (!Double.isFinite(ticksPerSecond) || ticksPerSecond < 0.0) {
            return;
        }
        if (targetVelocity != ticksPerSecond || !flywheelsOn) {
            readySince = 0;
            feedStarted = false;
            firing = false;
            if (fireRequested) fireRequestedAt = System.nanoTime();
        }
        targetVelocity = ticksPerSecond;
        flywheelsOn = true;
    }

    // Safe to call repeatedly from a held trigger; does not reset the timeout.
    public void fire() {
        if (!fireRequested) fireRequestedAt = System.nanoTime();
        fireRequested = true;
        ejecting = false;
    }





    public void actStopWheel() {
        fWheel.setPower(0);
    }


    /** Reverse directly; the saved forward velocity/toggle state is preserved. */
    public void outtake() {
        outtaking = true;
        readySince = 0;
        firing = false;
        requestedVolts = 0;
        motorPower = -OUTTAKE_POWER;
        fWheel.setPower(MOTOR0_POWER_SIGN * motorPower);
    }

    /** Stop only reverse output, independently of the forward velocity controller. */
    public void stopOuttake() {
        if (!outtaking) return;
        outtaking = false;
        readySince = 0;
        requestedVolts = motorPower = 0;
        fWheel.setPower(0);
    }


    public void eject(boolean requested) {
        ejecting = requested;
        if (ejecting) {
            fireRequested = false;
            firing = false;
            feedStarted = false;
            fireRequestedAt = 0;
        }
    }

    // Call once every OpMode loop, AFTER updating command/trigger requests.
    public void update() {
        if (outtaking) { outtake(); return; }
        long now = System.nanoTime();
        double velocity = ENCODER_SIGN_0 * fWheel.getVelocity();
        if (Double.isFinite(velocity)) measuredVelocity = velocity;
        batteryVoltage = readBatteryVoltage();

        requestedVolts = motorPower = 0.0;
        if (flywheelsOn && targetVelocity > 0.0) {
            double error = targetVelocity - measuredVelocity;
            requestedVolts = KS + KV * targetVelocity + KP * error;
            motorPower = clip(requestedVolts / batteryVoltage, 0.0, 1.0);
            if (Math.abs(error) <= VELOCITY_TOLERANCE) {
                if (readySince == 0) readySince = now;
            } else readySince = 0;

        } else {
            readySince = 0;
        }
        if (ejecting) motorPower = -.4;
        fWheel.setPower(clip(MOTOR0_POWER_SIGN * motorPower, -1, 1));

        // Gate the START of a firing request on stable speed. Once feeding
        // starts, keep feeding through normal shot dips, matching the ball test.
        if (fireRequested && flywheelsOn && !ejecting && !feedStarted) {
            if (flywheelAtSpeed()) feedStarted = true;
        }
        firing = fireRequested && flywheelsOn && feedStarted && !ejecting;

        //ball just shot logic
        if (measuredVelocity >= flywheelThreshold) {
            flywheelReady = true;
            flywheelDropped = false;
            ballJustShot = false;
        } else if (flywheelReady && !flywheelDropped) {
            flywheelDropped = true;
            ballJustShot = true;
        } else
            ballJustShot = false;
    }

    private double readBatteryVoltage() {
        double result = battery.get();
        return Double.isFinite(result) && result > 0 ? result : batteryVoltage;
    }

    private static double clip(double value, double low, double high) {
        return Double.isFinite(value) ? Math.max(low, Math.min(value, high)) : 0;
    }

    /** Compatibility API: automatic transfer fault latching is disabled. */
    public void clearFault() { }

    public void stopAll() {
        outtaking = false;
        flywheelsOn = fireRequested = firing = ejecting = feedStarted = false;
        targetVelocity = requestedVolts = motorPower = 0.0;
        readySince = fireRequestedAt = 0;
        fWheel.setPower(0.0);
    }

    public boolean flywheelAtSpeed() {
        return flywheelsOn && targetVelocity > 0.0
                && readySince != 0 && Math.abs(targetVelocity - measuredVelocity) <= VELOCITY_TOLERANCE
                && (System.nanoTime() - readySince) / 1e9 >= READY_HOLD_SECONDS;
    }

    public boolean isFlywheelsOn() { return flywheelsOn; }
    public boolean isFiring() { return firing; }
    public boolean isEjecting() { return ejecting; }
    public double getTargetVelocity() { return targetVelocity; }
    public double getError() { return targetVelocity - measuredVelocity; }
    public double getFlywheelVelo() { return fWheel.getVelocity(); }
    public double getMeasuredVelocity() { return measuredVelocity; }
    public double getMotorPower() { return motorPower; }
    public double getBatteryVoltage() { return batteryVoltage; }
    public boolean ballWasShot() {return ballJustShot;}
    public String getFault() { return ""; }
}
