package org.firstinspires.ftc.teamcode.trajectory.Functions;

/** Link to function: https://www.desmos.com/calculator/zlyb8ugqjm
 * @author TheConverseEngineer
 */
public class AccelQuartic implements ProfileFunction {

    private final double scalarCoeff;
    private final double veloFirstCo;
    private final double posFirstCo;
    private final double veloSecondCo;
    private final double posSecondCo;
    private final double veloK;
    private final double posK;

    public AccelQuartic(double accelTime, double maxVelo, double startVelo, double startPos) {
        this.scalarCoeff = maxVelo/0.0833;
        this.veloFirstCo = -1/(6*cube(accelTime));
        this.posFirstCo = this.veloFirstCo / 4;
        this.veloSecondCo = 1 / (4*square(accelTime));
        this.posSecondCo = this.veloSecondCo / 3;
        this.veloK = startVelo;
        this.posK = startPos;
    }

    @Override
    public double getVelocity(double t) {
        return scalarCoeff*((veloFirstCo * cube(t)) + (veloSecondCo * square(t))) + veloK;
    }

    @Override
    public double getPosition(double t) {
        return scalarCoeff*((posFirstCo * fourth(t)) + (posSecondCo * cube(t))) + veloK*t + posK;
    }

    private double fourth(double x) { return Math.pow(x, 4); }
    private double cube(double x) { return x * x * x; }
    private double square(double x) { return x * x; }
}
