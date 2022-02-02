package org.firstinspires.ftc.teamcode.src.robotAttachments.navigation.tfLocalization;

import org.firstinspires.ftc.teamcode.src.robotAttachments.navigation.LocalizationAlgorithm;
import org.firstinspires.ftc.teamcode.src.robotAttachments.navigation.odometry.IMUOdometry;
import org.firstinspires.ftc.teamcode.src.robotAttachments.navigation.odometry.Odometry;
import org.firstinspires.ftc.teamcode.src.robotAttachments.navigation.odometry.ThreeWheelOdometry;
import org.firstinspires.ftc.teamcode.src.robotAttachments.navigation.odometry.enums.FieldPoints;
import org.firstinspires.ftc.teamcode.src.utills.Executable;

public class VuforiaOdometryLA implements LocalizationAlgorithm {
    final Odometry odometry;
    final Executable<Double[]> getLocation;

    VuforiaOdometryLA(ThreeWheelOdometry odometry, Executable<Double[]> getLocation) {
        this.odometry = odometry;
        this.getLocation = getLocation;
    }

    public VuforiaOdometryLA(IMUOdometry odometry, Executable<Double[]> getLocation) {
        this.odometry = odometry;
        this.getLocation = getLocation;
    }

    @Override
    public double getX() {
        return this.getPos()[0];
    }

    @Override
    public double getY() {
        return this.getPos()[1];
    }

    @Override
    public double getRot() {
        return this.getPos()[2];
    }

    @Override
    public double[] getPos() {
        Double[] tmp = getLocation.call();
        if (tmp != null) {
            return new double[]{tmp[0], tmp[1], tmp[2]};
        } else {
            return odometry.getPos();
        }
    }

    @Override
    public void setPos(FieldPoints initPos) throws InterruptedException {
        double[] tmp = FieldPoints.positionsAndPoints.get(initPos);
        assert (tmp != null);
        this.setPos(tmp[0], tmp[1], tmp[2]);
    }

    @Override
    public void setPos(double X, double Y, double rot) throws InterruptedException {

    }
}
