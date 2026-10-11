package org.firstinspires.ftc.teamcode;

// path rules for the robot to follow
public interface HolonomicPath {
    Vec2 pos(double t);
    // Returns robot target coordinates (X, Y) along the path
    Vec2 vel(double t);
    // Returns target velocity vector (speed and direction)
    Vec2 acc(double t);
    // Returns target acceleration vector (speeding up/slowing down)

    String name();
    // Returns the type name of the path shape as text
    Vec2[] pts();
    // Returns the guide anchors used to shape the path
    void setPt(int idx, Vec2 p);
    // Moves a specific guide anchor to reshape the path

    // Calculates how sharp a turn is (higher number = tighter turn)
    default double curve(double t) {
        Vec2 v = vel(t); Vec2 a = acc(t);
        double sSq = v.x * v.x + v.y * v.y;
        double den = Math.pow(sSq, 1.5);
        if (den < 1e-9) return 0;
        return (v.x * a.y - v.y * a.x) / den;
    }

    // Finds the pure straight line direction the robot is facing along the path
    default Vec2 tan(double t) {
        Vec2 d = vel(t);
        double m = Math.hypot(d.x, d.y);
        if (m < 1e-9) return new Vec2(1, 0);
        return new Vec2(d.x / m, d.y / m);
    }

    // Same job as pos(t); kept so anything still calling the old name works.
    // An earlier version of this left pointAt as its own unimplemented method,
    // separate from pos(t), so none of the three classes below ever satisfied
    // it. That's why none of them could be built with plain new (...): a class
    // with an unimplemented interface method has to be marked abstract, and an
    // abstract class can never be instantiated directly, anywhere.
    default Vec2 pointAt(double t) { return pos(t); }
}

// Storable (X, Y) coordinate points or vector arrows
final class Vec2 {
    final double x, y;
    Vec2(double x1, double y1){
        x = x1; y = y1;
    }
    Vec2 add(Vec2 o){
// Adds vectors together
        return new Vec2(x + o.x, y + o.y);
    }
    Vec2 sub(Vec2 o){
// Subtracts vectors
        return new Vec2(x - o.x, y - o.y);
    }
    Vec2 scale(double s){
// Multiplies vector length
        return new Vec2(x * s, y * s);
    }
    double dot(Vec2 o){
// Measures alignment between 2 vectors
        return x * o.x + y * o.y;
    }
    double len(){
// Calculates total straight-line length
        return Math.hypot(x, y);
    }
    double dist(Vec2 o){
// Finds distance to another point
        return Math.hypot(x - o.x, y - o.y);
    }
    @Override public String toString() { return String.format("(%.2f, %.2f)", x, y); }
}

// A straight line path from a start point to an end point
class LinePath implements HolonomicPath {
    private Vec2 p0, p1; // Start and End points

    LinePath(Vec2 startPoint, Vec2 endPoint) {
        p0 = startPoint;
        p1 = endPoint;
    }

    @Override public Vec2 pos(double t){
        return p0.add(p1.sub(p0).scale(clamp(t)));
    }
    @Override public Vec2 vel(double t){
        return p1.sub(p0);
    } // Speed is constant all the way through
    @Override public Vec2 acc(double t){
        return new Vec2(0, 0);
    } // Straight lines have 0 acceleration
    @Override public String name(){
        return "Line";
    }
    @Override public Vec2[] pts(){
        return new Vec2[]{p0, p1};
    }
    @Override public void setPt(int i, Vec2 p){
        if (i == 0) p0 = p; else if (i == 1) p1 = p;
    }
    static double clamp(double t){
        return t < 0 ? 0 : (t > 1 ? 1 : t);
    } // Keeps timer scale bounded between 0 and 1
}

// A curved spline shaped by stretching lines toward pull anchors
class CubicBezierPath implements HolonomicPath {
    private final Vec2[] p = new Vec2[4]; // Array storing the 4 shape points

    CubicBezierPath(Vec2 p0, Vec2 p1, Vec2 p2, Vec2 p3) { p[0] = p0; p[1] = p1; p[2] = p2; p[3] = p3; }

    @Override public Vec2 pos(double t){
        t = LinePath.clamp(t); double u = 1 - t;
        double b0 = u*u*u, b1 = 3*u*u*t, b2 = 3*u*t*t, b3 = t*t*t;
        return new Vec2(
                b0*p[0].x + b1*p[1].x + b2*p[2].x + b3*p[3].x,
                b0*p[0].y + b1*p[1].y + b2*p[2].y + b3*p[3].y
        );
    }

    @Override public Vec2 vel(double t){
        t = LinePath.clamp(t); double u = 1 - t;
        Vec2 a = p[1].sub(p[0]).scale(3 * u * u);
        Vec2 b = p[2].sub(p[1]).scale(6 * u * t);
        Vec2 c = p[3].sub(p[2]).scale(3 * t * t);
        return a.add(b).add(c);
    }

    @Override public Vec2 acc(double t) {
        t = LinePath.clamp(t); double u = 1 - t;
        Vec2 a = p[0].add(p[2]).sub(p[1].scale(2)).scale(6 * u);
        Vec2 b = p[1].add(p[3]).sub(p[2].scale(2)).scale(6 * t);
        return a.add(b);
    }

    @Override public String name(){
        return "Cubic Bezier";
    }
    @Override public Vec2[] pts() {
        return new Vec2[]{p[0], p[1], p[2], p[3]};
    }
    @Override public void setPt(int i, Vec2 v) { if (i >= 0 && i < 4) p[i] = v; }
}

// Ultra smooth spline defined directly by speed and acceleration targets
class QuinticHermitePath implements HolonomicPath {
    private Vec2 p0, v0, a0, p1, v1, a1; // Start/End coordinates, velocities, and accelerations
    private double[] cx = new double[6]; // Horizontal math settings
    private double[] cy = new double[6]; // Vertical math settings

    QuinticHermitePath(Vec2 p0, Vec2 v0, Vec2 a0, Vec2 p1, Vec2 v1, Vec2 a1) {
        this.p0 = p0; this.v0 = v0; this.a0 = a0; this.p1 = p1; this.v1 = v1; this.a1 = a1;
        run();
    }

    // Creates a spline setting endpoints with directions but zero starting acceleration
    static QuinticHermitePath fromTans(Vec2 s, Vec2 st, Vec2 e, Vec2 et) {
        // No longer needs the anonymous override that used to sit here: that
        // override replaced the real position with a hardcoded null, so any
        // path built through this method would crash the moment anything
        // called .pointAt() on it. Removed now that pointAt isn't a separate
        // unimplemented method anymore.
        return new QuinticHermitePath(s, st,
                new Vec2(0, 0), e, et,
                new Vec2(0, 0));
    }

    // Refreshes the math curves whenever parameters change
    private void run() {
        cx = solve(p0.x, v0.x, a0.x, p1.x, v1.x, a1.x);
        cy = solve(p0.y, v0.y, a0.y, p1.y, v1.y, a1.y);
    }

    // Matrix calculation grid to blend position, velocity, and acceleration smoothly
    private static double[] solve(double p0, double v0, double a0, double p1, double v1, double a1) {
        double c0 = p0, c1 = v0, c2 = a0 / 2.0;
        double A = p1 - p0 - v0 - a0 / 2.0; double B = v1 - v0 - a0; double C = a1 - a0;
        return new double[]{c0, c1, c2, 10*A - 4*B + C/2.0, -15*A + 7*B - C, 6*A - 3*B + C/2.0};
    }

    @Override public Vec2 pos(double t) {
        t = LinePath.clamp(t); return new Vec2(eval(cx, t), eval(cy, t));
    }
    @Override public Vec2 vel(double t) {
        t = LinePath.clamp(t); return new Vec2(dEval(cx, t), dEval(cy, t));
    }
    @Override public Vec2 acc(double t) {
        t = LinePath.clamp(t); return new Vec2(ddEval(cx, t), ddEval(cy, t));
    }

    private static double eval(double[] c, double t)   {
        return c[0] + t * (c[1] + t * (c[2] + t * (c[3] + t * (c[4] + t * c[5]))));
    }
    private static double dEval(double[] c, double t)  {
        return c[1] + t * (2 * c[2] + t * (3 * c[3] + t * (4 * c[4] + t * 5 * c[5])));
    }
    private static double ddEval(double[] c, double t) {
        return 2 * c[2] + t * (6 * c[3] + t * (12 * c[4] + t * 20 * c[5]));
    }

    @Override public String name(){
        return "Quintic Hermite";
    }

    // Converts velocities to pull handles so the visual field map can display them properly
    @Override public Vec2[] pts() {
        return new Vec2[]{ p0, p0.add(v0.scale(0.33)), p1.sub(v1.scale(0.33)), p1 };
    }

    // Takes map interface drag adjustments and sets them back into the math structure
    @Override public void setPt(int i, Vec2 v) {
        if (i == 0)
            p0 = v;
        else if (i == 1)
            v0 = v.sub(p0).scale(1.0 / 0.33);
        else if (i == 2)
            v1 = p1.sub(v).scale(1.0 / 0.33);
        else if (i == 3)
            p1 = v;
        run();
    }
}

/**
 * here is my fav tame impala song for absolutely no reason other than....   (I contemplated if it's then or than for 10 minutes)
 * i am about to lose it!
 * Guess it from this lyric for bragging rights:
 *      Well, he feels like an elephant
 *      Shakin' his big grey trunk for the hell of it
 *      He knows that you're dreamin' about being loved by him
 *      Too bad your chances are slim (DUDU)
 * */