package org.firstinspires.ftc.teamcode.lib.physics;

import org.ejml.simple.SimpleMatrix;

public class MecanumDriveModel {
    private final int timeSteps;
    private final double dt;

    //Roller angles 2 and 3 are -45 degrees
    private final double robotMass;
    private final double wheelMass;
    private final double wheelInertiaSpinning;
    private final double wheelInertiaTurning;
    private final double robotMomentInertia;
    private final double wheelRadius;
    private final double internalGearRatio;
    private final double externalGearRatio;
    private final double compoundGearRatio;
    //Electrical constants
    private final double nominalVoltage;
    private final double stallTorque;
    private final double stallCurrent;
    private final double freeSpeed;
    private final double freeCurrent;
    private final double efficiency;

    private final double resistance;
    private final double kV;
    private final double kT;

    private final double wheelRadiusSquared;
    private final double wheelEffectiveSpinningMass;

    private final double L1;
    private final double L2;
    private final double D1;
    private final double D2;

    private CoefficientManager coefficientManager;

    public MecanumDriveModel(int timeSteps, double dt, double robotMass, double wheelMass, double wheelInertiaSpinning, double wheelInertiaTurning,
                             double robotMomentInertia, double wheelRadius, double internalGearRatio, double externalGearRatio, double nominalVoltage,
                             double stallTorque, double stallCurrent, double freeSpeed, double freeCurrent, double efficiency,
                             double L1, double L2, double D1, double D2) {
        this.timeSteps            = timeSteps;
        this.dt                   = dt;
        this.robotMass            = robotMass;
        this.wheelMass            = wheelMass;
        this.wheelInertiaSpinning = wheelInertiaSpinning;
        this.wheelInertiaTurning  = wheelInertiaTurning;
        this.robotMomentInertia   = robotMomentInertia;
        this.internalGearRatio    = internalGearRatio;
        this.externalGearRatio    = externalGearRatio;
        this.compoundGearRatio    = internalGearRatio * externalGearRatio;
        this.wheelRadius          = wheelRadius;
        this.nominalVoltage       = nominalVoltage;
        this.stallTorque          = efficiency * stallTorque;
        this.stallCurrent         = stallCurrent;
        this.freeSpeed            = internalGearRatio * freeSpeed;
        this.freeCurrent          = freeCurrent;
        this.efficiency           = efficiency;
        this.L1                   = L1;
        this.L2                   = L2;
        this.D1                   = D1;
        this.D2                   = D2;

        resistance = this.nominalVoltage / this.stallCurrent;
        kV         = (this.nominalVoltage - this.resistance * this.freeCurrent) / (this.freeSpeed);
        kT         = this.stallTorque / this.stallCurrent;

        wheelRadiusSquared         = this.wheelRadius * this.wheelRadius;
        wheelEffectiveSpinningMass = this.wheelInertiaSpinning / wheelRadiusSquared;
        coefficientManager = new CoefficientManager(this);
    }

    public static void main(String... args) {
        MecanumDriveModel model = new MecanumDriveModel(500, 0.001, 15.75d, 0.315d, 0.315d * (0.1 * 0.1 + 0.032 * 0.032) / 2,
                0.315d * (3 * (0.1 * 0.1 + 0.032 * 0.032) + 0.05 * 0.05) / 12, 0.5613d,
                0.1d / 2, 13.7d, 2d, 12d, 0.187d, 9.2d,
                435 * 2 * Math.PI / 60d, 0.25d, 0.6d,
                7d * 0.0254, 7d * 0.0254, 7d * 0.0254, 7d * 0.0254);

        SimpleMatrix state = new SimpleMatrix(6, 1, false, new double[] {
                0, 0, 0, 0, 0, 0
        });

        SimpleMatrix input = new SimpleMatrix(4, 1, false, new double[] {
                1, 0, 1, 0
        });

        System.out.println("Time\tx\tVx\ty\tVy\tpsi\tVpsi");
        System.out.println(0 + "\t" + state.get(0) + "\t" + state.get(1) + "\t" + state.get(2) + "\t" + state.get(3) + "\t" + state.get(4) + "\t" + state.get(5));
        for(int i = 1; i <= 3000; i++) {
            //state = model.simulate(state, input);
            state = model.stateTransitionMatrix(state, true).mult(state).plus(model.inputTransitionMatrix(state, false).mult(input));
            if(i % 10 == 0) {
                System.out.println((int)(i * model.dt * 100000) / 100000d + "\t" + state.get(0) + "\t" + state.get(1) + "\t" + state.get(2) + "\t" + state.get(3) +
                        "\t" + state.get(4) + "\t" + state.get(5));
            }
        }
    }

    public double A11(double heading) {
        return robotMass + 4 * wheelEffectiveSpinningMass;
    }

    public double A12(double heading) {
        return 0d;
    }

    public double A13(double heading) {
        return -2 * wheelMass * ((L1 - L2) * Math.sin(heading) + (D1 - D2) * Math.cos(heading)) +
                4 * wheelEffectiveSpinningMass * (
                        Math.sin(heading + Math.PI / 4) * (L2 - L1 + D2 - D1) -
                                Math.cos(heading + Math.PI / 4) * (L2 - L1 - D2 + D1)) / Math.sqrt(2);
    }

    public double A14(double heading) {
        return -A12(heading);
    }

    public double A15(double heading) {
        return 4 * wheelEffectiveSpinningMass;
    }

    public double A16(double heading) {
        return -2 * wheelMass * ((L1 - L2) * Math.cos(heading) - (D1 - D2) * Math.sin(heading));
    }

    public double A21(double heading) {
        return 0d;
    }

    public double A22(double heading) {
        return robotMass + 4 * wheelEffectiveSpinningMass;
    }

    public double A23(double heading) {
        return 2 * wheelMass * ((L1 - L2) * Math.cos(heading) - (D1 - D2) * Math.sin(heading)) -
                4 * wheelEffectiveSpinningMass * (
                        Math.cos(heading + Math.PI / 4) * (L2 - L1 + D2 - D1) +
                                Math.sin(heading + Math.PI / 4) * (L2 - L1 - D2 + D1)) / Math.sqrt(2);
    }

    public double A24(double heading) {
        return -A15(heading);
    }

    public double A25(double heading) {
        return A21(heading);
    }

    public double A26(double heading) {
        return -2 * wheelMass * ((L1 - L2) * Math.sin(heading) + (D1 - D2) * Math.cos(heading));
    }

    public double A31(double heading) {
        return A26(heading) + 4 * wheelEffectiveSpinningMass * (
                Math.sin(heading + Math.PI / 4) * (L2 - L1 + D2 - D1) -
                        Math.cos(heading + Math.PI / 4) * (L2 - L1 - D2 + D1)) / Math.sqrt(2);
    }

    public double A32(double heading) {
        return A23(heading);
    }

    public double A33(double heading) {
        double cosPsi = Math.cos(heading);
        double sinPsi = Math.sin(heading);
        return robotMomentInertia + 4 * wheelInertiaTurning + 2 * wheelEffectiveSpinningMass * (
                (L1 * cosPsi + D1 * sinPsi) * (L1 * cosPsi + D1 * sinPsi) +
                        (L1 * cosPsi - D2 * sinPsi) * (L1 * cosPsi - D2 * sinPsi) +
                        (L2 * cosPsi - D1 * sinPsi) * (L2 * cosPsi - D1 * sinPsi) +
                        (L2 * cosPsi + D2 * sinPsi) * (L2 * cosPsi + D2 * sinPsi));
    }

    public double A34(double heading) {
        return -2 * wheelMass * ((L1 - L2) * Math.cos(heading) + (D1 - D2) * Math.sin(heading)) +
                4 * wheelEffectiveSpinningMass * (
                        Math.cos(heading + Math.PI / 4) * (L2 - L1 + D2 - D1) +
                                Math.sin(heading + Math.PI / 4) * (L2 - L1 - D2 + D1)) / Math.sqrt(2);
    }

    public double A35(double heading) {
        return A26(heading) + 4 * wheelEffectiveSpinningMass * (
                Math.sin(heading + Math.PI / 4) * (L2 - L1 + D2 - D1) -
                        Math.cos(heading + Math.PI / 4) * (L2 - L1 - D2 + D1)) / Math.sqrt(2);
    }

    private double motorTorqueAccelerationX(SimpleMatrix torques, double heading) {
        double cosPsi = Math.cos(heading);
        double sinPsi = Math.sin(heading);
        return ((torques.get(0) + torques.get(3)) * (cosPsi + sinPsi) +
                (torques.get(1) + torques.get(2)) * (cosPsi - sinPsi)) / wheelRadius;
    }

    private double motorTorqueAccelerationY(SimpleMatrix torques, double heading) {
        double cosPsi = Math.cos(heading);
        double sinPsi = Math.sin(heading);
        return -((torques.get(0) + torques.get(3)) * (cosPsi - sinPsi) -
                (torques.get(1) + torques.get(2)) * (cosPsi + sinPsi)) / wheelRadius;
    }

    private double motorTorqueAccelerationHeading(SimpleMatrix torques, double heading) {
        return (torques.get(0) * (-L1 - D1) -
                torques.get(1) * (-L1 - D2) -
                torques.get(2) * (L2 + D1) +
                torques.get(3) * (L2 + D2)) / wheelRadius;
    }

    private SimpleMatrix getWheelVelocities(SimpleMatrix state) {
        double cosPsi = Math.cos(state.get(4));
        double sinPsi = Math.sin(state.get(4));
        return new SimpleMatrix(4, 1, false, new double[] {
                state.get(1) * (cosPsi + sinPsi) - state.get(3) * (cosPsi - sinPsi) - state.get(5) * (L1 + D1),
                state.get(1) * (cosPsi - sinPsi) + state.get(3) * (cosPsi + sinPsi) + state.get(5) * (L1 + D2),
                state.get(1) * (cosPsi - sinPsi) + state.get(3) * (cosPsi + sinPsi) - state.get(5) * (L2 + D1),
                state.get(1) * (cosPsi + sinPsi) - state.get(3) * (cosPsi - sinPsi) + state.get(5) * (L2 + D2),
        }).divide(wheelRadius);
    }

    private SimpleMatrix getMotorVelocities(SimpleMatrix state) {
        return getWheelVelocities(state).scale(externalGearRatio);
    }

    private SimpleMatrix getMotorTorques(SimpleMatrix state, SimpleMatrix input) {
        return input.scale(nominalVoltage).plus(getMotorVelocities(state).scale(internalGearRatio * kV)).scale(efficiency * internalGearRatio * kT / resistance);
    }

    public SimpleMatrix simulate(SimpleMatrix state, SimpleMatrix input) {
        SimpleMatrix torques = getMotorTorques(state, input);
        double heading = state.get(4);
        return state.plus(new SimpleMatrix(6, 6, false, new double[] {
                1, 0, 0, 0, 0, 0,
                0, A11(heading), 0, A21(heading), 0, A31(heading),
                0, 0, 1, 0, 0, 0,
                0, A12(heading), 0, A22(heading), 0, A32(heading),
                0, 0, 0, 0, 1, 0,
                0, A13(heading), 0, A23(heading), 0, A33(heading)
        }).invert().mult(new SimpleMatrix(6, 1, false, new double[] {
                state.get(1),
                motorTorqueAccelerationX(torques, heading) - state.get(5) * (A14(heading) * state.get(1) + A15(heading) * state.get(3) + A16(heading) * state.get(5)),
                state.get(3),
                motorTorqueAccelerationY(torques, heading) - state.get(5) * (A24(heading) * state.get(1) + A25(heading) * state.get(3) + A26(heading) * state.get(5)),
                state.get(5),
                motorTorqueAccelerationHeading(torques, heading) - state.get(5) * (A34(heading) * state.get(1) + A35(heading) * state.get(3))
        }).scale(dt)));
    }

    public SimpleMatrix simulate(SimpleMatrix state, SimpleMatrix input, double dt) {
        SimpleMatrix torques = getMotorTorques(state, input);
        double heading = state.get(4);

        coefficientManager.updateVariables(heading);
        double A11 = coefficientManager.get(1, 1);
        double A13 = coefficientManager.get(1, 3);
        double A14 = coefficientManager.get(1, 4);
        double A15 = coefficientManager.get(1, 5);
        double A16 = coefficientManager.get(1, 6);
        double A22 = coefficientManager.get(2, 2);
        double A23 = coefficientManager.get(2, 3);
        double A24 = coefficientManager.get(2, 4);
        double A25 = coefficientManager.get(2, 5);
        double A26 = coefficientManager.get(2, 6);
        double A31 = coefficientManager.get(3, 1);
        double A32 = coefficientManager.get(3, 2);
        double A33 = coefficientManager.get(3, 3);
        double A34 = coefficientManager.get(3, 4);
        double A35 = coefficientManager.get(3, 5);

        return state.plus(new SimpleMatrix(6, 6, false, new double[] {
                1,   0, 0,   0, 0,   0,
                0, A11, 1,   0, 0, A31,
                0,   0, 0,   0, 1,   0,
                0,   0, 0, A22, 0, A32,
                0,   0, 0,   0, 0,   0,
                0, A13, 0, A23, 0, A33
        }).invert().mult(new SimpleMatrix(6, 1, false, new double[] {
                state.get(1),
                motorTorqueAccelerationX(torques, heading) - state.get(5) * (A14 * state.get(1) + A15 * state.get(3) + A16 * state.get(5)),
                state.get(3),
                motorTorqueAccelerationY(torques, heading) - state.get(5) * (A24 * state.get(1) + A25 * state.get(3) + A26 * state.get(5)),
                state.get(5),
                motorTorqueAccelerationHeading(torques, heading) - state.get(5) * (A34 * state.get(1) + A35 * state.get(3))
        }).scale(dt)));
    }

    public SimpleMatrix simulateRungeKutta(SimpleMatrix state, SimpleMatrix input) {
        SimpleMatrix k1 = simulate(state, input, dt);
        SimpleMatrix k2 = simulate(state.plus(k1.scale(dt / 2)), input, dt / 2);
        SimpleMatrix k3 = simulate(state.plus(k2.scale(dt / 2)), input, dt / 2);
        SimpleMatrix k4 = simulate(state.plus(k3.scale(dt)), input, dt);
        return state.plus(k1.plus(k2.scale(2).plus(k3.scale(2).plus(k4))).scale(dt / 6));
    }

    public SimpleMatrix simulateRungeKutta(SimpleMatrix state, SimpleMatrix input, double dt) {
        SimpleMatrix k1 = simulate(state, input, dt);
        SimpleMatrix k2 = simulate(state.plus(k1.scale(dt / 2)), input, dt / 2);
        SimpleMatrix k3 = simulate(state.plus(k2.scale(dt / 2)), input, dt / 2);
        SimpleMatrix k4 = simulate(state.plus(k3.scale(dt)), input, dt);
        return state.plus(k1.plus(k2.scale(2).plus(k3.scale(2).plus(k4))).scale(dt / 6));
    }

    public SimpleMatrix stateTransitionMatrix(SimpleMatrix state, double dt, boolean updateCoefficients) {
        double xDot = state.get(1);
        double yDot = state.get(3);
        double heading = state.get(4);
        double psiDot = state.get(5);
        double cosPsi = Math.cos(heading);
        double sinPsi = Math.sin(heading);
        if(updateCoefficients) {
            coefficientManager.updateVariables(heading);
        }

        double A11 = coefficientManager.get(1, 1);
        double A13 = coefficientManager.get(1, 3);
        double A14 = coefficientManager.get(1, 4);
        double A15 = coefficientManager.get(1, 5);
        double A16 = coefficientManager.get(1, 6);
        double A22 = coefficientManager.get(2, 2);
        double A23 = coefficientManager.get(2, 3);
        double A24 = coefficientManager.get(2, 4);
        double A25 = coefficientManager.get(2, 5);
        double A26 = coefficientManager.get(2, 6);
        double A31 = coefficientManager.get(3, 1);
        double A32 = coefficientManager.get(3, 2);
        double A33 = coefficientManager.get(3, 3);
        double A34 = coefficientManager.get(3, 4);
        double A35 = coefficientManager.get(3, 5);

        final double v = (A13 * A22 * A31 + A11 * A23 * A32 - A11 * A22 * A33) * resistance * wheelRadius * wheelRadius;
        final double v11 = cosPsi * (D1 - D2) + (L1 - L2) * sinPsi;
        final double v12 = 2 * A16 * psiDot + A14 * xDot + A15 * yDot;
        final double v13 = D1 * D1 + D2 * D2 + L1 * (D2 + L1) + D2 * L2 + L2 * L2 + D1 * (L1 + L2);
        final double v1 = 2 * A11 * compoundGearRatio * compoundGearRatio * efficiency * kT * kV * v13 + 2 * A31 * compoundGearRatio * compoundGearRatio * efficiency * kT * kV * v11 -
                A31 * resistance * wheelRadius * wheelRadius * v12 + A11 * resistance * wheelRadius * wheelRadius * (A34 * xDot + A35 * yDot);
        final double v8 = cosPsi * L1 - cosPsi * L2 - D1 * sinPsi + D2 * sinPsi;
        final double v2 = 2 * compoundGearRatio * compoundGearRatio * efficiency * kT * kV * v8;
        final double v3 = v2 + resistance * wheelRadius * wheelRadius * (2 * A26 * psiDot + A24 * xDot + A25 * yDot);
        final double v4 = cosPsi * (L1 - L2) + (-D1 + D2) * sinPsi;
        final double v5 = -(cosPsi * D1) + cosPsi * D2 - L1 * sinPsi + L2 * sinPsi;
        final double v7 = 2 * A32 + A22 * cosPsi * (-L1 + L2) + A22 * (D1 - D2) * sinPsi;
        final double v9 = cosPsi * D1 - cosPsi * D2 + L1 * sinPsi - L2 * sinPsi;
        final double v6 = 2 * A31 + A11 * cosPsi * (D1 - D2) + A11 * (L1 - L2) * sinPsi;
        final double v10 = cosPsi * (-L1 + L2) + (D1 - D2) * sinPsi;

        double M11 = (2*compoundGearRatio*compoundGearRatio*efficiency*kT*kV*(-2*A23*A32 + 2*A22*A33 + A13*A22* v11) +
                (-(A14*A23*A32) + A13*A24*A32 + A14*A22*A33 - A13*A22*A34)*psiDot*resistance*wheelRadius*wheelRadius)/ v;
        double M13 = (2*A13*compoundGearRatio*compoundGearRatio*efficiency*kT*kV* v7 + (-(A15*A23*A32) + A13*A25*A32 + A15*A22*A33 - A13*A22*A35)*psiDot*resistance*wheelRadius*wheelRadius)/ v;
        double M14 = (2*compoundGearRatio*compoundGearRatio*efficiency*kT*kV*((A23*A32 - A22*A33)*psiDot* v4 +
                A13*(A32*psiDot* v5 + A22* v8 *xDot + A22* v9 *yDot)))/ v;
        double M15 = (A23*A32*(2*compoundGearRatio*compoundGearRatio*efficiency*kT*kV* v9 - resistance*wheelRadius*wheelRadius* v12) + A13*A32* v3 -
                A22*(2*A13*compoundGearRatio*compoundGearRatio*efficiency*kT*kV* v13 + 2*A33*compoundGearRatio*compoundGearRatio*efficiency*kT*kV* v11 -
                        A33*resistance*wheelRadius*wheelRadius* v12 + A13*resistance*wheelRadius*wheelRadius*(A34*xDot + A35*yDot)))/ v;
        double M31 = (2*A23*compoundGearRatio*compoundGearRatio*efficiency*kT*kV* v6 + (A14*A23*A31 - A13*A24*A31 + A11*A24*A33 - A11*A23*A34)*psiDot*resistance*
                wheelRadius*wheelRadius)/ v;
        double M33 = (2*compoundGearRatio*compoundGearRatio*efficiency*kT*kV*(-2*A13*A31 + 2*A11*A33 + A11*A23* v10) + (A15*A23*A31 - A13*A25*A31 + A11*A25*A33 - A11*A23*A35)*psiDot*resistance* wheelRadius*wheelRadius)/ v;
        double M34 = (2*compoundGearRatio*compoundGearRatio*efficiency*kT*kV*(A23*A31*psiDot* v10 + A13*A31*psiDot* v11 +
                A11*(A33*psiDot* v5 + A23* v8 *xDot + A23* v9 *yDot)))/ v;
        double M35 = -(((A13*A31 - A11*A33)* v3 + A23* v1)/ v);
        double M51 = -((2*A22*compoundGearRatio*compoundGearRatio*efficiency*kT*kV* v6 + (A14*A22*A31 + A11*A24*A32 - A11*A22*A34)*psiDot*resistance*wheelRadius*wheelRadius)/ v);
        double M53 = -((2*A11*compoundGearRatio*compoundGearRatio*efficiency*kT*kV* v7 + (A15*A22*A31 + A11*A25*A32 - A11*A22*A35)*psiDot*resistance*wheelRadius*wheelRadius)/ v);
        double M54 = (2*compoundGearRatio*compoundGearRatio*efficiency*kT*kV*(A22*A31*psiDot* v4 + A11*(A32*cosPsi*(D1 - D2)*psiDot + A32*(L1 - L2)*psiDot*sinPsi +
                A22*cosPsi*(-(L1*xDot) + L2*xDot - D1*yDot + D2*yDot) + A22*sinPsi*(D1*xDot - D2*xDot - L1*yDot + L2*yDot))))/ v;
        double M55 = (-(A11*A32* v3) + A22* v1)/ v;

        return new SimpleMatrix(6, 6, false, new double[] {
                0,   0, 0,   0, 0,   0,
                1, M11, 0, M31, 0, M51,
                0,   0, 0,   0, 0,   0,
                0, M13, 1, M33, 0, M53,
                0, M14, 0, M34, 0, M54,
                0, M15, 0, M35, 1, M55
        }).scale(dt).plus(SimpleMatrix.identity(6));
    }

    public SimpleMatrix stateTransitionMatrix(SimpleMatrix state, boolean updateCoefficients) {
        return stateTransitionMatrix(state, dt, updateCoefficients);
    }

    public SimpleMatrix inputTransitionMatrix(SimpleMatrix state, double dt, boolean updateCoefficients) {
        double xDot = state.get(1);
        double yDot = state.get(3);
        double heading = state.get(4);
        double psiDot = state.get(5);
        double cosPsi = Math.cos(heading);
        double sinPsi = Math.sin(heading);
        if(updateCoefficients) {
            coefficientManager.updateVariables(heading);
        }

        double A11 = coefficientManager.get(1, 1);
        double A13 = coefficientManager.get(1, 3);
        double A22 = coefficientManager.get(2, 2);
        double A23 = coefficientManager.get(2, 3);
        double A31 = coefficientManager.get(3, 1);
        double A32 = coefficientManager.get(3, 2);
        double A33 = coefficientManager.get(3, 3);

        final double v = (A13 * A22 * A31 + A11 * A23 * A32 - A11 * A22 * A33) * resistance * wheelRadius;
        final double v1 = (A23 * A32 - A22 * A33) * (cosPsi + sinPsi);
        final double v2 = (A23 * A32 - A22 * A33) * (cosPsi - sinPsi);
        double B10 = -((compoundGearRatio*efficiency*kT*nominalVoltage*(A13*A22*(D1 + L1) + A13*A32*(-cosPsi + sinPsi) - v1))/ v);
        double B11 = (compoundGearRatio*efficiency*kT*nominalVoltage*(A13*A22*(D2 + L1) + v2 - A13*A32*(cosPsi + sinPsi)))/ v;
        double B12 = -((compoundGearRatio*efficiency*kT*nominalVoltage*(A13*A22*(D1 + L2) - v2 + A13*A32*(cosPsi + sinPsi)))/ v);
        double B13 = (compoundGearRatio*efficiency*kT*nominalVoltage*(A13*A22*(D2 + L2) + A13*A32*(cosPsi - sinPsi) + v1))/ v;
        double B30 = -((compoundGearRatio*efficiency*kT*nominalVoltage*(A11*A23*(D1 + L1) + A13*A31*(cosPsi - sinPsi) + A11*A33*(-cosPsi + sinPsi) + A23*A31*(cosPsi + sinPsi)))/ v);
        double B31 = (compoundGearRatio*efficiency*kT*nominalVoltage*(A11*A23*(D2 + L1) + A23*A31*(-cosPsi + sinPsi) + A13*A31*(cosPsi + sinPsi) - A11*A33*(cosPsi + sinPsi)))/ v;
        double B32 = (compoundGearRatio*efficiency*kT*nominalVoltage*(-(A23*(A11*(D1 + L2) + A31*(cosPsi - sinPsi))) + A13*A31*(cosPsi + sinPsi) - A11*A33*(cosPsi + sinPsi)))/ v;
        double B33 = (compoundGearRatio*efficiency*kT*nominalVoltage*(A11*A23*(D2 + L2) + A11*A33*(cosPsi - sinPsi) + A13*A31*(-cosPsi + sinPsi) - A23*A31*(cosPsi + sinPsi)))/ v;
        double B50 = (compoundGearRatio*efficiency*kT*nominalVoltage*(A11*A22*(D1 + L1) + A11*A32*(-cosPsi + sinPsi) + A22*A31*(cosPsi + sinPsi)))/ v;
        double B51 = (compoundGearRatio*efficiency*kT*nominalVoltage*(-(A11*A22*(D2 + L1)) + A22*A31*(cosPsi - sinPsi) + A11*A32*(cosPsi + sinPsi)))/ v;
        double B52 = (compoundGearRatio*efficiency*kT*nominalVoltage*(A11*A22*(D1 + L2) + A22*A31*(cosPsi - sinPsi) + A11*A32*(cosPsi + sinPsi)))/ v;
        double B53 = (compoundGearRatio*efficiency*kT*nominalVoltage*(-(A11*A22*(D2 + L2)) + A11*A32*(-cosPsi + sinPsi) + A22*A31*(cosPsi + sinPsi)))/ v;

        return new SimpleMatrix(6, 4, false, new double[] {
                0, B10, 0, B30, 0, B50,
                0, B11, 0, B31, 0, B51,
                0, B12, 0, B32, 0, B52,
                0, B13, 0, B33, 0, B53
        }).scale(dt);
    }

    public SimpleMatrix inputTransitionMatrix(SimpleMatrix state, boolean updateCoefficients) {
        return inputTransitionMatrix(state, dt, updateCoefficients);
    }

    public SimpleMatrix simulateDynamics(SimpleMatrix state, SimpleMatrix input, double dt) {
        return stateTransitionMatrix(state, dt, true).mult(state).plus(inputTransitionMatrix(state, dt, false).mult(input));
    }

    public SimpleMatrix simulateDynamics(SimpleMatrix state, SimpleMatrix input) {
        return stateTransitionMatrix(state, true).mult(state).plus(inputTransitionMatrix(state, false).mult(input));
    }

    public int getTimeSteps() {
        return timeSteps;
    }

    public double getDt() {
        return dt;
    }

    public double getRobotMass() {
        return robotMass;
    }

    public double getWheelMass() {
        return wheelMass;
    }

    public double getWheelInertiaSpinning() {
        return wheelInertiaSpinning;
    }

    public double getWheelInertiaTurning() {
        return wheelInertiaTurning;
    }

    public double getRobotMomentInertia() {
        return robotMomentInertia;
    }

    public double getWheelRadius() {
        return wheelRadius;
    }

    public double getInternalGearRatio() {
        return internalGearRatio;
    }

    public double getExternalGearRatio() {
        return externalGearRatio;
    }

    public double getCompoundGearRatio() {
        return compoundGearRatio;
    }

    public double getNominalVoltage() {
        return nominalVoltage;
    }

    public double getStallTorque() {
        return stallTorque;
    }

    public double getStallCurrent() {
        return stallCurrent;
    }

    public double getFreeSpeed() {
        return freeSpeed;
    }

    public double getFreeCurrent() {
        return freeCurrent;
    }

    public double getEfficiency() {
        return efficiency;
    }

    public double getResistance() {
        return resistance;
    }

    public double getkV() {
        return kV;
    }

    public double getkT() {
        return kT;
    }

    public double getWheelRadiusSquared() {
        return wheelRadiusSquared;
    }

    public double getWheelEffectiveSpinningMass() {
        return wheelEffectiveSpinningMass;
    }

    public double getL1() {
        return L1;
    }

    public double getL2() {
        return L2;
    }

    public double getD1() {
        return D1;
    }

    public double getD2() {
        return D2;
    }
}
