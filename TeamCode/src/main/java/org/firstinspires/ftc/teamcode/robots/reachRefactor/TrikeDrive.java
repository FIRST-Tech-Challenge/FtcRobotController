package org.firstinspires.ftc.teamcode.robots.reachRefactor;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.acmerobotics.roadrunner.drive.Drive;
import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.kinematics.Kinematics;
import com.acmerobotics.roadrunner.localization.Localizer;

import org.firstinspires.ftc.teamcode.robots.reachRefactor.utils.UtilMethods;

import java.util.ArrayList;
import java.util.List;

public abstract class TrikeDrive extends Drive {

    private double trackWidth;
    private Localizer localizer;

    public TrikeDrive(double trackWidth, boolean simulated) {
        this.trackWidth = trackWidth;

        localizer = simulated ? new TrikeLocalizer(this, false) : new TrikeLocalizer(this, true);
    }

    class TrikeLocalizer implements Localizer {

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
                Pose2d robotPoseDelta = TrikeKinematics.wheelToRobotVelocities(wheelDeltas, drive.getTrackWidth());
                double finalHeadingDelta = useExternalHeading ? UtilMethods.wrapAngleRad(externalHeading - lastExternalHeading) : robotPoseDelta.getHeading();
                poseEstimate = Kinematics.relativeOdometryUpdate(poseEstimate, new Pose2d(robotPoseDelta.vec(), finalHeadingDelta));
            }

            List<Double> wheelVelocities = drive.getWheelVelocities();
            double externalHeadingVel = drive.getExternalHeadingVelocity();
            if(wheelVelocities != null) {
                poseVelocity = TrikeKinematics.wheelToRobotVelocities(wheelVelocities, drive.getTrackWidth());
                if(useExternalHeading)
                    poseVelocity = new Pose2d(poseVelocity.vec(), externalHeadingVel);
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

    public double getTrackWidth() {
        return trackWidth;
    }

    public abstract double getChassisLength();

    public abstract List<Double> getWheelPositions();

    public abstract List<Double> getWheelVelocities();

    public abstract void setSwivelAngle(double swivelAngle);

    public abstract void setMotorVelocities(double left, double right, double swerve);
}
