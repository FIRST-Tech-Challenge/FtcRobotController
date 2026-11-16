package org.firstinspires.ftc.teamcode.methods;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

public class Launcher {
    // Copy validated VOLTAGE-MODEL gains from LauncherAutoTuning here.
    // These are not REV Hub PIDFCoefficients. KV must be positive before use.
    // Gains validated in the 12-ball PI recovery test.
    public static double KS = 0.4006845, KV = 0.005114707;
    public static double KP = 0.032, KI = 0.060;
    public double SHOOTING_VELOCITY = 1480;

    /**
     * I am Maskin
     * Bow down to daddy
     * :P
     */
    // Copy the confirmed settings from the tuner. With both motors configured
    // FORWARD as below, the POWER_SIGN values must match the tuning OpMode.
    // ENCODER_SIGN_0 must match its learned "Encoder signs M0" value.
    public static double MOTOR0_POWER_SIGN = 1.0;
    public static double MOTOR1_POWER_SIGN = 1.0;
    public static double ENCODER_SIGN_0 = -1.0;
    public static double OUTTAKE_POWER = 0.40;

    // Transfer power is now a -1..1 duty-cycle setting, not ticks/second.
    public static double MECH_POWER = 1.0;
    // Starting examples: verify readiness settings on the robot.
    public static double VELOCITY_TOLERANCE = 40.0;
    public static double READY_HOLD_SECONDS = 0.15;
    /** Legacy setting retained for compatibility; readiness timeouts are disabled. */
    public static double FEED_READY_TIMEOUT_SECONDS = 3.0;

    private final BatteryVoltage battery;
    private final DcMotorEx fWheel;
    private final FlywheelIntegralControl flywheelController = new FlywheelIntegralControl();
    private boolean flywheelsOn = false;
    private boolean outtaking;
    private boolean fireRequested = false, firing = false, ejecting = false;
    private boolean feedStarted = false;
    private double targetVelocity = 0.0, measuredVelocity = 0.0;
    private double batteryVoltage = 12.76, requestedVolts = 0.0, motorPower = 0.0;
    private long lastControlTime = 0;
    private long readySince = 0, fireRequestedAt = 0;
    private boolean ballJustShot = false;
    private boolean flywheelDropped = false;
    // only count drops once the flywheel has been up to speed (ignores spin-up)
    private boolean flywheelReady = false;
    private double flywheelThreshold = 1200;
    private double distanceFromHive = 0;

    public Launcher(HardwareMap hardwareMap) {
        this(hardwareMap, new BatteryVoltage(hardwareMap));
    }

    /** Preferred: share one BatteryVoltage between Launcher, Transfer and Intake. */
    public Launcher(HardwareMap hardwareMap, BatteryVoltage battery) {
        this(hardwareMap.get(DcMotorEx.class, "fWheel"), battery);
    }

    Launcher(DcMotorEx motor, Iterable<VoltageSensor> sensors) {
        this(motor, new BatteryVoltage(sensors));
    }

    Launcher(DcMotorEx motor, BatteryVoltage battery) {
        this.battery = battery;
        fWheel = motor;
        configureFlywheel(fWheel);
        //stopAll();
    }

    private void configureFlywheel(DcMotorEx motor) {
        motor.setPower(0.0);
        motor.setDirection(DcMotorSimple.Direction.FORWARD);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    public void toggleOn() {
        if (!flywheelsOn) {
            setTargetVelocity(SHOOTING_VELOCITY);
        }
        else {
            stopAll();
        }

    }

    public void setTargetVelocity(double ticksPerSecond) {
        if (!Double.isFinite(ticksPerSecond) || ticksPerSecond < 0.0) {
            return;
        }
        if (outtaking) return;
        if (Math.abs(targetVelocity - ticksPerSecond) > VELOCITY_TOLERANCE || !flywheelsOn) {
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


    public void stopFire() {
        fireRequested = false;
        firing = false;
        feedStarted = false;
        fireRequestedAt = 0;
    }

    public void actStopWheel() {
        fWheel.setPower(0);
    }

    /** Reverse directly without changing the saved forward target or Y-toggle state. */
    public void outtake() {
        outtaking = true;
        readySince = 0;
        flywheelController.reset();
        firing = false;
        requestedVolts = 0;
        motorPower = -OUTTAKE_POWER;
        fWheel.setPower(MOTOR0_POWER_SIGN * motorPower);
    }

    /** Stop only an active reverse command. Normal forward update() resumes separately. */
    public void stopOuttake() {
        if (!outtaking) return;
        outtaking = false;
        readySince = 0;
        flywheelController.reset();
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
    // robotX/robotY: current robot pose from the follower, used for distance-based velocity.
    public void update(double robotX, double robotY, double amplitude, int yint) {
        updateDistanceTarget(robotX, robotY);
        update();
    }

    /** Prepare the distance target without writing motor power; coordinator performs the update. */
    public void updateDistanceTarget(double robotX, double robotY) {
        //distance to the closer hive target by pose
        double d1 = Math.hypot(robotX - 58, robotY - 50);
        double d2 = Math.hypot(robotX - 58, robotY - 90);
        distanceFromHive = Math.min(d1, d2);

        SHOOTING_VELOCITY = -70.3 * Math.cos(1.0 / 22 * distanceFromHive + 11.0 / 21) + 1407;

        //updating the speed
        if (flywheelsOn)
            setTargetVelocity(SHOOTING_VELOCITY);

    }

    // Same as above but without pose: keeps the current target velocity.
    public void update() {
        if (outtaking) { outtake(); return; }
        long now = System.nanoTime();
        double velocity = ENCODER_SIGN_0 * fWheel.getVelocity();
        if (Double.isFinite(velocity)) measuredVelocity = velocity;
        batteryVoltage = readBatteryVoltage();

        requestedVolts = motorPower = 0.0;
        if (flywheelsOn) {
            double error = targetVelocity - measuredVelocity;
            double dt = lastControlTime == 0 ? 0.02 : (now - lastControlTime) / 1e9;
            if (!Double.isFinite(dt) || dt <= 0 || dt > 0.1) {
                flywheelController.reset();
                dt = 0.02;
            }
            requestedVolts = flywheelController.volts(targetVelocity, measuredVelocity,
                    dt, batteryVoltage, KS, KV, KP, KI);
            motorPower = FlywheelVelocityControl.power(requestedVolts, batteryVoltage);
            if (Math.abs(error) <= VELOCITY_TOLERANCE) {
                if (readySince == 0) readySince = now;
            } else readySince = 0;

        } else {
            readySince = 0;
            flywheelController.reset();
        }
        lastControlTime = now;
        fWheel.setPower(clip(MOTOR0_POWER_SIGN * motorPower, -1, 1));

        // Gate the START of a firing request on stable speed. Once feeding
        // starts, keep feeding through normal shot dips, matching the ball test.
        if (fireRequested && flywheelsOn && !ejecting && !feedStarted) {
            if (flywheelAtSpeed()) feedStarted = true;
        }
        firing = fireRequested && flywheelsOn && feedStarted && !ejecting;

        //ball just shot logic (signed-corrected speed; the raw encoder reads negative)
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
        // Missing voltage data does not latch a fault or stop operator commands.
        return Double.isFinite(result) && result > 0 ? result : batteryVoltage;
    }

    private static double clip(double value, double low, double high) {
        return Double.isFinite(value) ? Math.max(low, Math.min(value, high)) : 0;
    }

    /** Compatibility API: automatic launcher fault latching is disabled. */
    public void clearFault() { }

    public void stopAll() {
        outtaking = false;
        flywheelsOn = fireRequested = firing = ejecting = feedStarted = false;
        targetVelocity = requestedVolts = motorPower = 0.0;
        readySince = fireRequestedAt = 0;
        lastControlTime = 0;
        flywheelController.reset();
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
    /** Signed-corrected flywheel speed from the last update() (positive when shooting). */
    public double getFlywheelVelo() { return measuredVelocity; }
    public double getMeasuredVelocity() { return measuredVelocity; }
    public double getMotorPower() { return motorPower; }
    /** Unclipped controller demand, before conversion to available battery power. */
    public double getRequestedVolts() { return requestedVolts; }
    public double getIntegralVolts() { return flywheelController.integralVolts(); }
    /** SDK-reported commanded power; this does not prove physical motor motion. */
    public double getAppliedMotorPower() { return fWheel.getPower(); }
    public double getBatteryVoltage() { return batteryVoltage; }
    public boolean ballWasShot() {return ballJustShot;}
    public String getFault() { return ""; }
    public double getDistanceFromHive() { return distanceFromHive; }
}
