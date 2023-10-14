package org.firstinspires.ftc.teamcode.lib.physics;

import org.firstinspires.ftc.teamcode.lib.geometry.Rotation2d;

public class CoefficientManager {
    private static MecanumDriveModel model;

    private double[] coefficients;
    private double heading;

    public CoefficientManager(MecanumDriveModel model) {
        CoefficientManager.model = model;
    }

    public CoefficientManager(MecanumDriveModel model, double heading) {
        CoefficientManager.model = model;
        updateVariables(heading);
    }

    public CoefficientManager(MecanumDriveModel model, Rotation2d heading) {
        this(model, heading.getRadians());
    }

    public double get(int row, int column) {
        if(--row * 6 + --column >= coefficients.length) {
            return 0d;
        }

        return coefficients[row * 6 + column];
    }

    public void updateVariables(double heading) {
        this.heading = heading;
        updateVariables();
    }

    public void updateVariables() {
        coefficients = new double[] {
                model.A11(heading), model.A12(heading), model.A13(heading),
                model.A14(heading), model.A15(heading), model.A16(heading),
                model.A21(heading), model.A22(heading), model.A23(heading),
                model.A24(heading), model.A25(heading), model.A26(heading),
                model.A31(heading), model.A32(heading), model.A33(heading),
                model.A34(heading), model.A35(heading)
        };
    }
}
