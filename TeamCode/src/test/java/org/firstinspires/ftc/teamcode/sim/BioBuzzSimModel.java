package org.firstinspires.ftc.teamcode.sim;

import org.horizon36596.simloop.config.DrivetrainSimConfig;
import org.horizon36596.simloop.plant.MecanumDrivePlant;

/**
 * Independent motor response and chassis motion with the reported two-stage drive reduction.
 * Nominal wheel diameter is reported hardware; chassis geometry, strafe ratio and lag
 * are synthetic estimates, NOT robot tuning.
 * Assertions check signs/relative motion, never measured speed, stopping distance, or shot success.
 */
final class BioBuzzSimModel implements DrivetrainSimConfig {
    // Independent hardware specification, deliberately not copied from the production constants.
    static final double MOTOR_TICKS_PER_REV = 28;
    static final double FREE_SPEED_TICKS_PER_SECOND = MOTOR_TICKS_PER_REV * 5800.0 / 60;
    static final double MOTOR_REVS_PER_WHEEL_REV = (80.0 / 20) * (91.0 / 30);
    static final double WHEEL_RADIUS_IN = 104.0 / 2 / 25.4; // Team-reported 104-mm goBILDA 30A wheel.
    static final double RESPONSE_RATE_PER_SECOND = 10;
    static final String PARAMETERS = "seed=none; drive motor=goBILDA 5000-0002-4008,28 ticks/rev,5800 RPM catalog-tested @12V; "
            + "drive transmission=20:80 gears,coaxial 30:91 belt,364/30 motor rev/wheel rev; "
            + "wheel=goBILDA 30A mecanum,104 mm diameter; motor ticks/in=28*(80/20)*(91/30)/(pi*104/25.4); "
            + "SYNTHETIC: track=14 in,wheelbase=14 in,"
            + "strafe ratio=0.85,lag=0.1 s; mounting signs=+1,-1,+1,-1 (team reversal report 2026-11-05); "
            + "flywheel=5203 Yellow Jacket,observed 5400 RPM at 14 V,28 ticks/rev,1:1 output; "
            + "turret=355 deg/servo span,8192 CPR*108/40,synthetic slew=180 deg/s; "
            + "Pinpoint=32,-74 mm,reversed pods,2000/(32pi) ticks/mm; "
            + "camera=synthetic FOV54x41,30Hz,35ms latency; "
            + "SYNTHETIC slide=CR power*521.74 deg/s shaft (rated Axon MAX+ 6 V),no load/lag,starts 300 deg; "
            + "no contact/ball/Lynx model";
    final MecanumDrivePlant drive;
    final BioBuzzSimHardware hardware;
    double yawDegrees, yawVelocity;
    // Unwrapped slide servo shaft angle; independent of production constants.
    double slideShaftDeg = BioBuzzSimHardware.SLIDE_START_SHAFT_DEG;
    boolean jamSlide;
    boolean jamTurret;

    BioBuzzSimModel(BioBuzzSimHardware hardware) {
        this.hardware = hardware;
        drive = hardware.wheels[0] == null ? null : new MecanumDrivePlant(hardware.wheels, this);
    }

    void update(double seconds) {
        // OpModeRunner already steps every fake device once before calling this method.
        if (drive != null) drive.update(seconds);
        if (hardware.yaw1 != null) {
            double target = (.5 - hardware.yaw1.getScaledPosition()) * 355;
            double delta = jamTurret ? 0 : Math.max(-180 * seconds, Math.min(180 * seconds, target - yawDegrees));
            yawDegrees += delta; yawVelocity = delta / seconds;
            hardware.encoder.setEncoderMeasuringAnotherMechanism(yawDegrees * 61.44, yawVelocity * 61.44);
        }
        if (hardware.slideServo != null) {
            if (!jamSlide) slideShaftDeg += hardware.slideServo.getPhysicalPower() * (60.0 / .115) * seconds;
            double wrapped = ((slideShaftDeg % 360) + 360) % 360;
            hardware.slideEncoder.setVoltage(wrapped / 360 * hardware.slideEncoder.getMaxVoltage());
        }
        double x = drive == null ? 0 : drive.getX();
        double y = drive == null ? 0 : drive.getY();
        double heading = drive == null ? 0 : drive.getHeading();
        if (hardware.pinpoint != null) hardware.pinpoint.moveChassisTo(x * 25.4, y * 25.4, heading);
        if (hardware.camera != null) hardware.camera.setRobotPose(
                x + 50.68184865 / 25.4 * Math.cos(heading),
                y + 50.68184865 / 25.4 * Math.sin(heading), heading + Math.toRadians(yawDegrees) - Math.PI / 2);
    }

    @Override public double trackWidth() { return 14; }
    @Override public double wheelBase() { return 14; }
    @Override public double wheelRadius() { return WHEEL_RADIUS_IN; }
    @Override public double ticksPerInch() {
        return MOTOR_TICKS_PER_REV * MOTOR_REVS_PER_WHEEL_REV / (2 * Math.PI * WHEEL_RADIUS_IN);
    }
    @Override public double maxVelocityTicksPerSecond() { return FREE_SPEED_TICKS_PER_SECOND; }
    @Override public double maxLateralVelocityTicksPerSecond() { return FREE_SPEED_TICKS_PER_SECOND * .85; }
    @Override public double maxAccel() { return RESPONSE_RATE_PER_SECOND; }
    // Team reported all six drive directions reversed on 2026-11-05. This replaces the old
    // assumed mount; keep physical shaft-to-wheel signs independent of motor.getDirection().
    @Override public double[] wheelMountingSigns() { return new double[] {1, -1, 1, -1}; }
    @Override public double fieldHalfWidth() { return 72; }
    @Override public double fieldHalfHeight() { return 72; }
}
