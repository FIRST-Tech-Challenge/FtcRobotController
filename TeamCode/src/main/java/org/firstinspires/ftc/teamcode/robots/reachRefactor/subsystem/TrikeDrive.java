package org.firstinspires.ftc.teamcode.robots.reachRefactor.subsystem;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.acmerobotics.roadrunner.drive.Drive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.Localizer;

import org.firstinspires.ftc.teamcode.robots.reachRefactor.util.TrikeKinematics;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.teamcode.robots.reachRefactor.util.Constants.*;
import static org.firstinspires.ftc.teamcode.robots.reachRefactor.util.Utils.*;

/**
 * @author Mahesh Natamai
 */

public abstract class TrikeDrive extends Drive {

    private Localizer localizer;

    public TrikeDrive(boolean simulated) {
        localizer = simulated ? new TrikeLocalizer(this, false) : new TrikeLocalizer(this, true);
    }

    static class TrikeLocalizer implements Localizer {

        private TrikeDrive drive;
        private Pose2d poseEstimate;
        private Pose2d poseVelocity;
        private List<Double> lastWheelPositions;
        private double lastExternalHeading;
        private boolean useExternalHeading;

        public TrikeLocalizer(TrikeDrive drive, boolean useExternalHeading) {
            this.drive = drive;
            this.useExternalHeading = useExternalHeading;
            lastWheelPositions = new ArrayList<>();
        }

        @Override
        public void update() {
            List<Double> wheelPositions = drive.getWheelPositions();
            double externalHeading = useExternalHeading ? drive.getExternalHeading() : Double.NaN;
            if(!lastWheelPositions.isEmpty()) {
                List<Double> wheelDeltas = new ArrayList<>();
                for(int i = 0; i < wheelPositions.size(); i++) {
                    wheelDeltas.add(wheelPositions.get(i) - lastWheelPositions.get(i));
                }

                double displacement = (wheelDeltas.get(0) + wheelDeltas.get(1)) / 2;
                double heading = useExternalHeading ?
                        wrapAngleRad(poseEstimate.getHeading() + wrapAngleRad(externalHeading - lastExternalHeading)) :
                        wrapAngleRad(poseEstimate.getHeading() + wrapAngleRad((-wheelDeltas.get(0) + wheelDeltas.get(1)) / TRACK_WIDTH));
                poseEstimate = new Pose2d(
                        poseEstimate.getX() + displacement * Math.cos(heading),
                        poseEstimate.getY() + displacement * Math.sin(heading),
                        heading
                );
            }

            List<Double> wheelVelocities = drive.getWheelVelocities();
            double externalHeadingVel = drive.getExternalHeadingVelocity();
            if(wheelVelocities != null) {
                    poseVelocity = useExternalHeading ?
                            new Pose2d((wheelVelocities.get(0) + wheelVelocities.get(1)) / 2, 0, externalHeadingVel) :
                            new Pose2d((wheelVelocities.get(0) + wheelVelocities.get(1)) / 2, 0, (-wheelVelocities.get(0) + wheelVelocities.get(1)) / TRACK_WIDTH);
            }

            lastWheelPositions = wheelPositions;
            lastExternalHeading = externalHeading;
        }

        @NonNull
        @Override
        public Pose2d getPoseEstimate() {
            return poseEstimate;
        }

        public void setPoseEstimate(Pose2d poseEstimate) {
            lastWheelPositions = new ArrayList<>();
            lastExternalHeading = Double.NaN;
            if(useExternalHeading)
                drive.setExternalHeading(poseEstimate.getHeading());
            this.poseEstimate = poseEstimate;
        }

        @Nullable
        @Override
        public Pose2d getPoseVelocity() {
            return poseVelocity;
        }
    }

    @NonNull
    @Override
    public Localizer getLocalizer() {
        return localizer;
    }

    @Override
    public void setLocalizer(@NonNull Localizer localizer) {
        this.localizer = localizer;
    }

    public abstract double getChassisLength();

    public abstract List<Double> getWheelPositions();

    public abstract List<Double> getWheelVelocities();

    public abstract void setSwivelAngle(double swivelAngle);

    public abstract void setMotorVelocities(double left, double right, double swerve);
}
