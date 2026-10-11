package org.firstinspires.ftc.teamcode;

public class LQRPathFollower {

    private final double maxV;
    private final double maxA;
    private final double maxAngV;

    private final double vHead;

    private final double[][] K;

    private static final double INTEGRAL_GAIN = 0.4;
    private static final double INTEGRAL_CLAMP = 4.0;
    private static final double SATURATION_THRESHOLD = 0.99;

    private double integralX = 0, integralY = 0, integralTheta = 0;
    private boolean saturated = false;

    private HolonomicPath path;
    private ArcLengthTable arcTable;
    private TrapezoidalProfile profile;

    private double elapsed = 0;
    private double latencySeconds = 0.0;
    private ChassisDynamics dynamics = null;

    static final double NOMINAL_BATTERY = 12.0;
    private double prevVxRobot = 0, prevVyRobot = 0, prevOmega = 0;
    private boolean hasPrevCommand = false;
    private Reference lastReference = null;

    private double sH;
    private double eH;

    public LQRPathFollower(double maxVe, double maxAcc, double maxAngVe,
                           double q, double r, double qTheta, double rTheta, double dt, double veHead) {

        if (maxVe <= 0 || maxAcc <= 0 || maxAngVe <= 0) {
            throw new IllegalArgumentException("velocity and acceleration limits must be > 0");
        }
        if (veHead <= 0 || veHead > 1) {
            throw new IllegalArgumentException("velocityHeadroom must be in (0, 1]");
        }

        maxV = maxVe;
        maxA = maxAcc;
        maxAngV = maxAngVe;
        vHead = veHead;

        double[][] Q = {
                {q, 0, 0},
                {0, q, 0},
                {0, 0, qTheta}
        };
        double[][] R = {
                {r, 0, 0},
                {0, r, 0},
                {0, 0, rTheta}
        };

        K = solveLQR(dt, Q, R);
    }

    public static LQRPathFollower withDefaults(double maxVel, double maxAccel, double maxOmega) {
        return new LQRPathFollower(
                maxVel, maxAccel, maxOmega,
                10.0, 1.0, 8.0, 1.0, 0.02, 0.7
        );
    }

    public void followPath(HolonomicPath path, double sH, double eH) {
        this.path = path;
        arcTable = new ArcLengthTable(path);

        profile = new TrapezoidalProfile(arcTable.totalLength(), maxV * vHead, maxA);

        this.sH = sH;
        this.eH = eH;
        elapsed = 0;

        integralX = integralY = integralTheta = 0;
        saturated = false;
        hasPrevCommand = false;
        prevVxRobot = prevVyRobot = prevOmega = 0;
    }

    public Reference referenceAt(double t) {
        double s = profile.posAt(t);
        double speed = profile.vAt(t);
        double param = arcTable.tAtArcLength(s);

        Vec2 pos = path.pos(param);
        Vec2 tangent = path.tan(param);

        double total = profile.duration();
        double frac = total < 1e-9 ? 1.0 : clamp01(t / total);

        double delta = normalizeAngle(eH - sH);
        double heading = sH + delta * frac;

        boolean withinProfile = (t > 0) && (t < total);
        double omega = withinProfile ? delta / total : 0.0;

        return new Reference(
                pos,
                tangent.scale(speed),
                heading,
                omega,
                s,
                path.curve(param)
        );
    }

    public double[] update(double measuredX, double measuredY, double measuredHeading, double dt) {
        return update(measuredX, measuredY, measuredHeading, dt, NOMINAL_BATTERY);
    }

    public double[] update(Localizer localizer, double dt, double batteryVoltage) {
        Pose pose = localizer.getPose();
        return update(pose.x, pose.y, pose.h, dt, batteryVoltage);
    }

    public double[] update(double measuredX, double measuredY, double measuredHeading, double dt, double batteryVoltage) {
        if (path == null) {
            return new double[]{0, 0, 0, 0};
        }

        elapsed += dt;
        Reference ref = referenceAt(elapsed);

        Reference ffRef = latencySeconds > 0 ? referenceAt(elapsed + latencySeconds) : ref;

        double errX = ref.p.x - measuredX;
        double errY = ref.p.y - measuredY;
        double errTheta = normalizeAngle(ref.h - measuredHeading);

        double fbX = K[0][0] * errX + K[0][1] * errY + K[0][2] * errTheta;
        double fbY = K[1][0] * errX + K[1][1] * errY + K[1][2] * errTheta;
        double fbTheta = K[2][0] * errX + K[2][1] * errY + K[2][2] * errTheta;

        integralX = accumulate(integralX, errX, dt);
        integralY = accumulate(integralY, errY, dt);
        integralTheta = accumulate(integralTheta, errTheta, dt);

        double vxField = ffRef.v.x + fbX + INTEGRAL_GAIN * integralX;
        double vyField = ffRef.v.y + fbY + INTEGRAL_GAIN * integralY;
        double omega   = ffRef.o + fbTheta + INTEGRAL_GAIN * integralTheta;

        lastReference = ref;

        if (dynamics != null) {
            return toWheelPowersViaDynamics(vxField, vyField, omega, measuredHeading, dt, batteryVoltage);
        }
        return toWheelPowers(vxField, vyField, omega, measuredHeading);
    }

    private double[] toWheelPowersViaDynamics(double vxField, double vyField, double omega, double heading, double dt, double batteryVoltage) {
        double c = Math.cos(heading), s = Math.sin(heading);
        double vxRobot =  vxField * c + vyField * s;
        double vyRobot = -vxField * s + vyField * c;
        double ax = 0, ay = 0, alpha = 0;

        if (hasPrevCommand && dt > 1e-6) {
            ax = (vxRobot - prevVxRobot) / dt;
            ay = (vyRobot - prevVyRobot) / dt;
            alpha = (omega - prevOmega) / dt;
        }

        prevVxRobot = vxRobot;
        prevVyRobot = vyRobot;
        prevOmega = omega;
        hasPrevCommand = true;

        double[] powers = dynamics.toMotorPowers(vxRobot, vyRobot, omega, ax, ay, alpha, batteryVoltage);

        double rawMax = 0;
        for (double p : powers) {
            rawMax = Math.max(rawMax, Math.abs(p));
        }

        saturated = rawMax > SATURATION_THRESHOLD;
        return powers;
    }

    private double[] toWheelPowers(double vxField, double vyField, double omega, double heading) {
        double c = Math.cos(heading), s = Math.sin(heading);
        double vxRobot =  vxField * c + vyField * s;
        double vyRobot = -vxField * s + vyField * c;

        double vx = vxRobot / maxV;
        double vy = vyRobot / maxV;
        double w  = omega / maxAngV;

        // vy points LEFT and w is counterclockwise here (same as the path and the odometry),
        // so these are the right handed mecanum signs: left strafe = FL back, FR forward, BL forward, BR back;
        // counterclockwise = left wheels back, right wheels forward.
        // The old signs (vx + vy + w ...) are the stick style ones where vy is RIGHT and w is clockwise,
        // which made lateral and heading feedback push away from the path.
        double[] p = new double[4];
        p[0] = vx - vy - w;
        p[1] = vx + vy + w;
        p[2] = vx + vy - w;
        p[3] = vx - vy + w;

        double rawMax = 0;
        for (double v : p) {
            rawMax = Math.max(rawMax, Math.abs(v));
        }
        saturated = rawMax > SATURATION_THRESHOLD;

        double divisor = Math.max(1.0, rawMax);
        for (int i = 0; i < 4; i++) {
            p[i] /= divisor;
        }
        return p;
    }

    private double accumulate(double current, double error, double dt) {
        boolean unwinding = (current * error) < 0;
        if (saturated && !unwinding) {
            return current;
        }
        double next = current + error * dt;
        return Math.max(-INTEGRAL_CLAMP, Math.min(INTEGRAL_CLAMP, next));
    }

    public double[] brake() {
        path = null;
        profile = null;
        return new double[]{0, 0, 0, 0};
    }

    public boolean isFinished() {
        return profile != null && elapsed >= profile.duration();
    }

    public double duration() {
        return profile == null ? 0 : profile.duration();
    }

    public double elapsed() {
        return elapsed;
    }

    public boolean isSaturated() {
        return saturated;
    }

    public double pathLength() {
        return arcTable == null ? 0 : arcTable.totalLength();
    }

    public double[][] gains() {
        return K;
    }

    public void setLatencyCompensation(double seconds) {
        if (seconds < 0) {
            throw new IllegalArgumentException("latency must be >= 0");
        }
        if (seconds > 0.15) {
            throw new IllegalArgumentException("latency preview too high, measure real loop delay instead");
        }
        latencySeconds = seconds;
    }

    public void setChassisDynamics(ChassisDynamics d) {
        dynamics = d;
    }

    public boolean hasDynamics() {
        return dynamics != null;
    }

    public Reference lastReference() {
        return lastReference;
    }

    public int currentSegment() {
        if (!(path instanceof PathChain) || profile == null) {
            return 0;
        }

        double distanceTraveled = profile.posAt(elapsed);
        double totalDistance = arcTable.totalLength();

        double distanceFraction;
        if (totalDistance < 1e-9) {
            distanceFraction = 1;
        } else {
            distanceFraction = clamp01(distanceTraveled / totalDistance);
        }

        return ((PathChain) path).segmentAt(distanceFraction);
    }

    public double profiledMaxV() {
        return maxV * vHead;
    }

    public void resyncTo(double measuredX, double measuredY) {
        if (path == null) {
            return;
        }
        double s = arcTable.closestArcLength(path, new Vec2(measuredX, measuredY));
        elapsed = profile.timeAtPosition(s);
    }

    private static double[][] solveLQR(double dt, double[][] Q, double[][] R) {
        double[][] K = new double[3][3];
        for (int i = 0; i < 3; i++) {
            K[i][i] = solveScalar(dt, Q[i][i], R[i][i]);
        }
        return K;
    }

    public static double scalarLqrGain(double dt, double q, double r) {
        return solveScalar(dt, q, r);
    }

    private static double solveScalar(double b, double q, double r) {
        if (b <= 0) throw new IllegalArgumentException("dt must be > 0");
        if (q <= 0) throw new IllegalArgumentException("Q must be > 0");
        if (r <= 0) throw new IllegalArgumentException("R must be > 0 (R must be +)");

        double p = (q * b + Math.sqrt(q * q * b * b + 4 * q * r)) / (2 * b);
        if (Double.isNaN(p) || Double.isInfinite(p)) {
            throw new ArithmeticException("Riccati solve produced a non-finite solution");
        }

        double residual = q - (p * p * b * b) / (r + p * b * b);
        if (Math.abs(residual) > 1e-6 * Math.max(1.0, q)) {
            throw new ArithmeticException(String.format("Riccati solve failed residual check"));
        }
        return (b * p) / (r + p * b * b);
    }

    static double normalizeAngle(double a) {
        while (a > Math.PI) a -= 2 * Math.PI;
        while (a < -Math.PI) a += 2 * Math.PI;
        return a;
    }

    static double clamp01(double v) {
        return v < 0 ? 0 : (v > 1 ? 1 : v);
    }

    public static class Reference {
        public final Vec2 p;
        public final Vec2 v;
        public final double h;
        public final double o;
        public final double arcL;
        public final double curv;

        Reference(Vec2 position, Vec2 velocity, double heading, double om, double arcLength, double curvature) {
            p = position;
            v = velocity;
            h = heading;
            o = om;
            arcL = arcLength;
            curv = curvature;
        }
    }
}