package org.firstinspires.ftc.teamcode.lib.control;

public class ControlConstants {
    private final double kP;
    private final double kI;
    private final double kD;

    private final double kS;
    private final double kV;
    private final double kA;

    public ControlConstants() {
        this(0, 0, 0, 0, 0, 0);
    }

    public ControlConstants(final double kP) {
        this(kP, 0, 0, 0, 0, 0);
    }

    public ControlConstants(final double kV, final double kA) {
        this(0, 0, 0, 0, kV, kA);
    }

    public ControlConstants(final double kP, final double kI, final double kD) {
        this(kP, kI, kD, 0, 0, 0);
    }

    public ControlConstants(final double kP, final double kI, final double kD,
                            final double kS, final double kV, final double kA) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kS = kS;
        this.kV = kV;
        this.kA = kA;
    }

    public ControlConstants(ControlConstants controlConstants) {
        this.kP = controlConstants.kP();
        this.kI = controlConstants.kI();
        this.kD = controlConstants.kD();
        this.kS = controlConstants.kS();
        this.kV = controlConstants.kV();
        this.kA = controlConstants.kA();
    }

    public double getOutput(double dt, double error, double lastError, double runningSum, double setpointVelocity, double setpointAcceleration, boolean staticFriction) {
        double output = kP() * error + kI() * runningSum + kD() * ((error - lastError) / dt - setpointVelocity) +
                kV() * setpointVelocity + kA() * setpointAcceleration;
        return output + (staticFriction ? kS() * Math.signum(output) : 0d);
    }

    public double kP() {
        return kP;
    }

    public double kI() {
        return kI;
    }

    public double kD() {
        return kD;
    }

    public double kS() {
        return kS;
    }

    public double kV() {
        return kV;
    }

    public double kA() {
        return kA;
    }

    @Override
    public String toString() {
        return "kP = " + kP() + ", kI = " + kI() + ", kD = " + kD() + ", kS = " + kS() + ", kV = " + kV() + ", kA = " + kA();
    }
}
